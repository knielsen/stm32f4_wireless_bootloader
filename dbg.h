extern void setup_serial(void);
extern void serial_putchar(uint32_t c);
extern void serial_puts(const char *s);
extern void serial_output_hexbyte(uint8_t byte);
extern void println_uint32(uint32_t val);
extern void println_int32(int32_t val);
extern void print_uint32_hex(uint32_t val);
extern void println_float(float f, uint32_t dig_before, uint32_t dig_after);
extern void serial_dump_buf(uint8_t *buf, uint32_t len);
