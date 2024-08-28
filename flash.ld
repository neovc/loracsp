MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 256K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

PROVIDE(__app_start__ = ORIGIN(rom));
PROVIDE(__app_size__ = _data_loadaddr + _edata - _data - __app_start__);
PROVIDE(__app_code_size__ = _etext - __app_start__);
PROVIDE(__app_ram_size__ = _ebss - ORIGIN(ram));
