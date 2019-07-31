MEMORY
{
	/* Restrict FLASH to the first 16K block for the bootloader. */
	FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 16K
	RAM_MAIN (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
	RAM_CPU (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}
_estack = 0x10010000;    /* end of 64K RAM internal in Cortex M4 */

/*
   We could set up a .bss_xxx section for RAM_MAIN.
   But since we only need a single buffer in there, it's probably easier to
   just hard-code the address in the .c file.
*/


SECTIONS
{
	.text :
	{
		KEEP(*(.isr_vector))
		*(.text*)
		*(.rodata*)
		*(.init*)
		*(.fini*)
		. = ALIGN(4);
		_etext = .;
	} > FLASH

	_sidata = .;

	.data : AT(_sidata)
	{
		_sdata = .;
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} > RAM_CPU

	.bss :
	{
		_sbss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} > RAM_CPU

	/* Check that there is room for stack. */
	._stack_free_space :
	{
		. = . + 1K;  /* Minimum 1 kByte stack free. */
	} > RAM_CPU
}

ENTRY(Reset_Handler)
