/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000 + 0K, LENGTH = 128K - 12K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld

