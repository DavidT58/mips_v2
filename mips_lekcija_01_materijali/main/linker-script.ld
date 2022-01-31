MEMORY
{
	REGION (rwx) : ORIGIN = 0x0000EE00, LENGTH = 32k
}

SECTIONS
{
	prva :
	{
		. = . + 5;
		BYTE(0xFF)
		LONG(0x01234567)
		. = ALIGN(16);
		*(.text)
	} > REGION
	
	druga 0x66660000 :
	{
		*(.rodata)
	}
	
	treca 0x77770000 :
	{
		*(.data)
	}
	
	cetvrta 0x88880000 :
	{
		*(COMMON)
	}
	
	peta :
	{
		*(.comment)
		*(.ARM.attributes)
	}
}