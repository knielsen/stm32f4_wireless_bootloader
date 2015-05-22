/* POV3D. */

#include <string.h>
#include <stm32f4_discovery.h>

#include "dbg.h"
#include "led.h"
#include "nrf24l01p.h"


/* Start offset for target main program. */
static const uint32_t TARGET_START = 0x08004000;
static const uint32_t TARGET_END = 0x08100000;
static const uint32_t FLASH_START = 0x08000000;


/* Communications protocol. */
#define POV_CMD_DEBUG 254
#define POV_SUBCMD_RESET_TO_BOOTLOADER 255
#define POV_SUBCMD_ENTER_BOOTLOADER 254
#define POV_SUBCMD_RESET_TO_APP 253
#define POV_SUBCMD_FLASH_BUFFER 252
#define POV_SUBCMD_EXIT_DEBUG   251
#define POV_SUBCMD_STATUS_REPLY 240


/* This is apparently needed for libc/libm (eg. powf()). */
int __errno;


/* To change this, must change the code in system_stm32f4xx.c. */
#define MCU_HZ 168000000


static void
delay(__IO uint32_t nCount)
{
    while(nCount--)
        __asm("nop"); // do nothing
}


static inline void
bzero(void *buf, uint32_t len)
{
  memset(buf, 0, len);
}


/*
  Setup SPI communication for nRF42L01+ on USART1 in synchronous mode.

    PA8   clk
    PB6   mosi
    PB7   miso
    PC6   cs
    PC7   ce
    PC8   irq
*/
static void
setup_nrf_spi(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  USART_ClockInitTypeDef USART_ClockInitStruct;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  USART_Cmd(USART1, DISABLE);

  /*
    Clock on PA8.
    Polarity is idle low, active high.
    Phase is sample on rising, setup on falling edge.
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_USART1);

  /* MOSI and MISO on PB6/PB7. */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

  /*
    CS on PC6, CE on PC7 (we do not need IRQ in this bootloader).
    CS is high initially (active low).
    CE is low initially (active high).
  */
  GPIO_SetBits(GPIOC, GPIO_Pin_6);
  GPIO_ResetBits(GPIOC, GPIO_Pin_7);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 2000000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  USART_Init(USART1, &USART_InitStructure);
  USART_ClockInitStruct.USART_Clock = USART_Clock_Enable;
  USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStruct.USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStruct.USART_LastBit = USART_LastBit_Enable;
  USART_ClockInit(USART1, &USART_ClockInitStruct);

  USART_Cmd(USART1, ENABLE);
}


static inline void
csn_low(void)
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_6);
}


static inline void
csn_high(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_6);
}


static inline void
ce_low(void)
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_7);
}


static inline void
ce_high(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_7);
}


static inline uint8_t
bitswap_byte(uint8_t in)
{
  return (uint8_t)(__RBIT((uint32_t)in) >> 24);
}


static void
ssi_cmd(uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len)
{
  uint32_t i;

  /* Take CSN low to initiate transfer. */
  csn_low();

  /*
    Do the transfer, writing bytes and reading the returned bytes.
    Note that nRF SPI uses most-significant-bit first, while USART works
    with least-significant-bit first. So we need to bit-swap all the
    bytes sent and received.
  */
  for (i = 0; i < len; ++i)
  {
    while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE))
      ;
    USART_SendData(USART1, bitswap_byte(sendbuf[i]));
    while (!USART_GetFlagStatus(USART1, USART_FLAG_RXNE))
      ;
    recvbuf[i] = bitswap_byte(USART_ReceiveData(USART1));
  }

  /* Take CSN high to complete transfer. */
  csn_high();
}


static void
nrf_rx(uint8_t *data, uint32_t len)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > 32)
    len = 32;
  sendbuf[0] = nRF_R_RX_PAYLOAD;
  bzero(&sendbuf[1], len);
  ssi_cmd(recvbuf, sendbuf, len+1);
  memcpy(data, &recvbuf[1], len);
}


static void
nrf_tx(uint8_t *data, uint32_t len)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > 32)
    len = 32;
  sendbuf[0] = nRF_W_TX_PAYLOAD;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1);
}


static void
nrf_flush_tx(void)
{
  uint8_t cmd = nRF_FLUSH_TX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1);
}


static void
nrf_flush_rx(void)
{
  uint8_t cmd = nRF_FLUSH_RX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1);
}


static void
nrf_write_reg_n(uint8_t reg, const uint8_t *data, uint32_t len)
{
  uint8_t sendbuf[6], recvbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1);
}


static void
nrf_write_reg(uint8_t reg, uint8_t val)
{
  nrf_write_reg_n(reg, &val, 1);
}


static void
nrf_read_reg_n(uint8_t reg, uint8_t *out, uint32_t len)
{
  uint8_t sendbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_R_REGISTER | reg;
  bzero(&sendbuf[1], len);
  ssi_cmd(out, sendbuf, len+1);
}


static uint8_t
nrf_read_reg(uint8_t reg, uint8_t *status_ptr)
{
  uint8_t recvbuf[2];
  nrf_read_reg_n(reg, recvbuf, 2);
  if (status_ptr)
    *status_ptr = recvbuf[0];
  return recvbuf[1];
}


static const uint8_t nrf_addr[3] = { 0xe7, 0xe7, 0xe7 };


/*
  Configure nRF24L01+ as Rx or Tx.
    channel - radio frequency channel to use, 0 <= channel <= 127.
    power - nRF_RF_PWR_<X>DBM, <X> is 0, 6, 12, 18 dBm.
*/
static void
nrf_init_config(uint8_t is_rx, uint32_t channel, uint32_t power)
{
  if (is_rx)
    nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP);
  else
    nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5);
  /* Enable only pipe 0. */
  nrf_write_reg(nRF_EN_RXADDR, nRF_ERX_P0);
  /* 3 byte adresses. */
  nrf_write_reg(nRF_SETUP_AW, nRF_AW_3BYTES);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15);
  nrf_write_reg(nRF_RF_CH, channel);
  /* Use 2Mbps, and set transmit power. */
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_HIGH | power);
  nrf_write_reg_n(nRF_RX_ADDR_P0, nrf_addr, 3);
  nrf_write_reg_n(nRF_TX_ADDR, nrf_addr, 3);
  /* Set payload size for pipe 0. */
  nrf_write_reg(nRF_RX_PW_P0, 32);
  /* Disable pipe 1-5. */
  nrf_write_reg(nRF_RX_PW_P1, 0);
  /* Disable dynamic payload length. */
  nrf_write_reg(nRF_DYNDP, 0);
  /* Allow disabling acks. */
  nrf_write_reg(nRF_FEATURE, nRF_EN_DYN_ACK);

  /* Clear out all FIFOs. */
  nrf_flush_tx();
  nrf_flush_rx();
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT);
}


/*
  Configure nRF24L01+ as Tx for replying status from the bootloader.

  The nRF24L01+ is configured as transmitter, with auto-ack and
  retransmission enabled.
*/
static void
nrf_config_bootload_tx()
{
  nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15);

  /* Clear out all FIFOs. */
  nrf_flush_tx();
  nrf_flush_rx();
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT);
}


/*
  Configure nRF24L01+ as Rx for getting commands for the bootloader.

  The nRF24L01+ is configured as receiver, with auto-ack and
  retransmission enabled.
*/
static void
nrf_config_bootload_rx(void)
{
  nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15);

  /* Clear out all FIFOs. */
  nrf_flush_tx();
  nrf_flush_rx();
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT);
}


/*
  Would use SysTick_Config(), but it has a bug that it does not allow to
  set the reload value to 0xffffff (off-by-one error, max is 0xfffffe).
  It also enables systicks interrupt, which we do not want here.
*/
static void
setup_systick(void)
{
  SysTick->LOAD = 0xffffff;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


static inline uint32_t
get_time(void)
{
  return SysTick->VAL;
}


static inline uint32_t
calc_time_from_val(uint32_t start, uint32_t stop)
{
  return (start - stop) & 0xffffff;
}


static inline uint32_t
calc_time(uint32_t start)
{
  uint32_t stop = get_time();
  return calc_time_from_val(start, stop);
}


static inline uint32_t
dec_time(uint32_t val, uint32_t inc)
{
  return (val - inc) & 0xffffff;
}


/*
  Delay until specified amount of systicks have passed.

  As systick is a 24-bit counter, the amount cannot exceed 0xffffff, or a bit
  more than 16000000.
*/
static void
delay_systicks(uint32_t cycles)
{
  uint32_t start = get_time();

  while (calc_time(start) < cycles)
    ;
}


static void
delay_us(uint32_t us)
{
  /* This assumes that MCU_HZ is divisible by 1000000. */
  uint32_t cycles = (MCU_HZ/1000000)*us;
#if (MCU_HZ % 1000000)
#error delay_us() computes delay incorrectly if MCU_HZ is not a multiple of 1000000
#endif

  while (cycles > 0xff0000)
  {
    delay_systicks(0xff0000);
    cycles -= 0xff0000;
  }
  delay_systicks(cycles);
}


/* Buffer holding one block of flash data from client. */
static uint32_t flash_buffer[1024/sizeof(uint32_t)];

static uint32_t * const sector_data = (uint32_t *)0x20000000;
static uint32_t sector_data_idx = 0;
static const uint32_t sector_sizes[12] = {
   16*1024,  16*1024,  16*1024,  16*1024,  64*1024, 128*1024,
  128*1024, 128*1024, 128*1024, 128*1024, 128*1024, 128*1024
};
static const uint32_t sector_offsets[12] = {
      0,         16*1024,    32*1024,    48*1024,    64*1024,   128*1024,
  2*128*1024, 3*128*1024, 4*128*1024, 5*128*1024, 6*128*1024, 7*128*1024
};


/*
  Write any pending flash data.

  The wireless flash tools were originally written for the Tiva TM4C, where
  the flash is simply an array of 1024 byte pages, so the protocol is written
  to match that. However, the STM32F4 has a complex mix of large sector sizes.
  So we have to buffer the 1024 byte chunks, and write them out to flash as
  each flash sector is filled.

  We check if we need to erase a sector before writing - it is only necessary
  if we have to change a '0' bit to a '1'.

  Sector 0 is special; it holds the bootloader, and we never overwrite it.
*/
static void
flush_sector_data(void)
{
  uint32_t idx = sector_data_idx;
  uint32_t sector_size;
  uint32_t dest_addr;
  uint32_t do_write = 0, do_erase = 0;
  uint32_t i;
  FLASH_Status status;

  if (!idx)
    return;
  sector_size = sector_sizes[idx];
  dest_addr = FLASH_START + sector_offsets[idx];

  /* Check if we need erase. */
  for (i = 0; i < sector_size/sizeof(uint32_t); ++i)
  {
    uint32_t old = ((uint32_t *)dest_addr)[i];
    uint32_t new = sector_data[i];
    if (~old & new)
      do_erase = 1;
    if (old != new)
      do_write = 1;
  }
  if (do_erase || do_write)
    FLASH_Unlock();
  if (do_erase)
  {
    serial_puts("Erase sector ");
    println_uint32(idx);
    status = FLASH_EraseSector(idx<<3, VoltageRange_3);
    if (status != FLASH_COMPLETE)
    {
      serial_puts("Error during flash erase: ");
      println_uint32((uint32_t)status);
    }
  }
  if (do_write)
  {
    serial_puts("Write sector ");
    println_uint32(idx);
    for (i = 0; i < sector_size/sizeof(uint32_t); ++i)
    {
      uint32_t new = sector_data[i];
      if (((uint32_t *)dest_addr)[i] != new)
      {
        status = FLASH_ProgramWord(dest_addr + i*sizeof(uint32_t), new);
        if (status != FLASH_COMPLETE)
        {
          serial_puts("Error during flash write: ");
          println_uint32((uint32_t)status);
          break;
        }
      }
    }
  }

  if (do_erase || do_write)
    FLASH_Lock();

  /* Mark that data has been flushed. */
  sector_data_idx = 0;
}


/*
  Invoke the real target application.

  We need to relocate the interrupt vector, load the stack pointer from
  vector 0, and then jump to vector 1.
*/
static void
jump_to_target(uint32_t target_start)
{
  uint32_t stack = ((uint32_t *)target_start)[0];
  uint32_t start = ((uint32_t *)target_start)[1];

  flush_sector_data();

  /* De-initialise the nRF24L01+, powering it down. */
  nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_RX_DR | nRF_MASK_TX_DS |
                nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO);
  csn_high();
  ce_low();

  /* ToDo: maybe de-initialise some stuff:
      - systicks
      - GPIOs
      - USARTs
  */

  serial_puts("Boot to target\r\n");
for (;;) { /* ToDo */}

  SCB->VTOR = target_start;
  __asm__ __volatile__
    ("mov  sp, %0\n\t"
     "bx   %1\n"
     :
     : "r" (stack), "r" (start)
     );
}


/*
  Read both the normal and FIFO status registers.
  Returns normal status or'ed with (fifo status left-shifted 8).
*/
static uint32_t
nrf_get_status(void)
{
  uint8_t status;
  uint32_t fifo_status;

  fifo_status = nrf_read_reg(nRF_FIFO_STATUS, &status);
  return (fifo_status << 8) | status;
}


static uint32_t
nrf_transmit_packet(uint8_t *packet)
{
  uint32_t start_time = get_time();

  nrf_tx(packet, 32);
  ce_high();
  delay_us(10);
  ce_low();

  for (;;)
  {
    uint32_t status = nrf_get_status();
    if (status & nRF_MAX_RT)
    {
      serial_puts("No ack from receiver\r\n");
      return 1;
    }
    if (status & nRF_TX_DS)
      return 0;
    if (calc_time(start_time) > 16000000)
    {
      serial_puts("Timeout from nRF24L01+ waiting for transmit\r\n");
      return 1;
    }
  }
}


static void
nrf_send_status_reply(uint8_t *packet_buf, uint8_t status)
{
  ce_low();
  nrf_config_bootload_tx();
  bzero(packet_buf, 32);
  packet_buf[0] = POV_CMD_DEBUG;
  packet_buf[1] = POV_SUBCMD_STATUS_REPLY;
  packet_buf[2] = status;
  if (nrf_transmit_packet(packet_buf))
    jump_to_target(TARGET_START);
  nrf_config_bootload_rx();
  ce_high();
}


static uint32_t
addr_to_block_idx(uint32_t addr)
{
  uint32_t i;
  uint32_t base;

  base = FLASH_START;
  if (addr < base)
    return 0;
  for (i = 0; i < 12; ++i)
  {
    uint32_t sector_size = sector_sizes[i];
    base += sector_size;
    if (addr < base)
      return i;
  }
  return 0;
}


static void
accept_blok_data(uint32_t addr, uint32_t *data)
{
  uint32_t idx = addr_to_block_idx(addr);
  uint32_t offset;

  if (idx != sector_data_idx)
  {
    /* New block (needs flush of data), or the very first block. */
    if (sector_data_idx)
      flush_sector_data();
    sector_data_idx = idx;
    memset(sector_data, 0xff, sector_sizes[idx]);
  }
  offset = addr - sector_offsets[idx] - FLASH_START;
  memcpy((uint8_t *)sector_data + offset, data, 1024);
}


int
main(void)
{
  uint32_t start_time, wait_counter;
  uint8_t status;
  uint8_t val;
  uint8_t packet_buf[32];

  setup_systick();
  setup_led();
  setup_serial();

  serial_puts("\r\n\r\nSTM32F4 wireless bootloader\r\nCopyright 2015 Kristian Nielsen\r\n");

  setup_nrf_spi();
  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  delay(168000000/3/10);

  nrf_init_config(0 /* Tx */, 2, nRF_RF_PWR_0DBM);
  serial_puts("Tx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status);
  serial_output_hexbyte(val);
  serial_puts(" status=0x");
  serial_output_hexbyte(status);
  serial_puts("\r\n");
  serial_puts("Wireless bootloader started\r\n");

  nrf_config_bootload_rx();
  /* Assert CE to start receiving. */
  ce_high();

  start_time = get_time();
  /*
    Systick can only count up to just under 0.1 seconds. We want to wait for
    longer for pov_sender to contact us, so count in 0.05-seconds intervals.

    (I prefer not to keep the bootloader as simple as possible, thus no timer
    interrupt).
  */
  wait_counter = 40;  /* 2 seconds */
  while (wait_counter > 0)
  {
    uint32_t now_time;
    uint32_t status = nrf_get_status();
    if (status & nRF_RX_DR)
      break;                                    /* Data ready. */
    now_time = get_time();
    if (calc_time_from_val(start_time, now_time) > MCU_HZ/20)
    {
      --wait_counter;
      start_time = dec_time(start_time, MCU_HZ/20);
      serial_puts(".");
    }
  }
  serial_puts("\r\n");

  /* If no packet received within 2 seconds, proceed to boot target app. */
  if (!wait_counter)
  {
    serial_puts("No programmer found\r\n");
    jump_to_target(TARGET_START);
  }

  /*
    Check if we got a "hello" packet. If we did, send a reply and start
    processing bootloader commands. But if not, just boot to the target
    application.
  */
  nrf_rx(packet_buf, 32);
  if (packet_buf[0] != POV_CMD_DEBUG ||
      packet_buf[1] != POV_SUBCMD_ENTER_BOOTLOADER)
  {
    serial_puts("Got unexpected packet\r\n");
    jump_to_target(TARGET_START);
  }
  /* Now reply with a status packet so sender knows we are here. */
  serial_puts("Got request from programmer, sending ack\r\n");
  /*
    For some reason this delay is needed. Else we time out waiting for the
    TX_DS flag.
  */
  delay_us(100);
  nrf_send_status_reply(packet_buf, 0);
  serial_puts("Ack sent, starting to process cmds\r\n");

  memset(flash_buffer, 0xff, sizeof(flash_buffer));
  /* Now loop, processing received commands. */
  for (;;)
  {
    /* Clear the "data ready flag", then check fifo to avoid races. */
    nrf_write_reg(nRF_STATUS, nRF_RX_DR);
    /* Read any packets in the Rx fifo. */
    while (!(nRF_RX_EMPTY & nrf_read_reg(nRF_FIFO_STATUS, NULL)))
    {
      uint32_t do_reply;
      uint8_t reply_status;
      uint8_t subcmd;

      nrf_rx(packet_buf, 32);
      if (packet_buf[0] != POV_CMD_DEBUG)
      {
        serial_puts("Got non-debug packet\r\n");
        jump_to_target(TARGET_START);
      }

      subcmd = packet_buf[1];
      //serial_puts("P:");
      //println_uint32(subcmd);
      if (subcmd <= 34)
      {
        /* Load flash data. */
        uint32_t start = subcmd * 30;
        uint32_t len = 30;
        if (start + len > 1024)
          len = 1024 - start;
        memcpy((uint8_t *)flash_buffer + start, &packet_buf[2], len);
        do_reply = 0;
      }
      else if (subcmd == POV_SUBCMD_FLASH_BUFFER)
      {
        uint32_t block;
        block = packet_buf[2] |
          ((uint32_t)packet_buf[3] << 8) |
          ((uint32_t)packet_buf[4] << 16) |
          ((uint32_t)packet_buf[5] << 24);
        /*
          Hm. Originally the wireless flasher was only for Tiva TM4C, where the
          flash starts at address 0 and the application (after bootload) starts
          at 0x1000.

          But on STM32F4, the flash starts at 0x08000000, and the application
          starts at 0x08004000.

          The wireless flash script is hard-coded to start at block 4, that
          should perhaps be fixed, but for now, let's just map block 4 to the
          start of the application in the flash.
        */
        block = block + 12 + 0x08000000/1024;
        if (block < TARGET_START/1024 || block >= TARGET_END/1024)
        {
          serial_puts("Invalid block to flash\r\n");
          reply_status = 1;
        }
        else
        {
          accept_blok_data(block*1024, flash_buffer);
          reply_status = 0;
        }
        /* Clear the buffer in case of partial load of next flash data. */
        memset(flash_buffer, 0xff, sizeof(flash_buffer));
        do_reply = 1;
      }
      else if (subcmd == POV_SUBCMD_RESET_TO_APP ||
               subcmd == POV_SUBCMD_EXIT_DEBUG)
      {
        serial_puts("Remote requested boot of target app\r\n");
        jump_to_target(TARGET_START);
        /* NotReached. */
        do_reply = 0;
      }
      else if (subcmd == POV_SUBCMD_ENTER_BOOTLOADER)
      {
        /* Apparently flasher started over, let's send an ack and proceed. */
        memset(flash_buffer, 0xff, sizeof(flash_buffer));
        reply_status = 0;
        do_reply = 1;
      }
      else
      {
        do_reply = 0;
      }

      if (do_reply)
      {
        nrf_send_status_reply(packet_buf, reply_status);
        serial_puts("*");
      }
    }
    /* Wait for more data to arrive. */
    while (!(nRF_RX_DR & nrf_get_status()))
      ;
  }
}
