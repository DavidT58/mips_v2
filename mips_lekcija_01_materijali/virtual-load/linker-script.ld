MEMORY
{
	REGION_PRVI (rwx) : ORIGIN = 0x22220000, LENGTH = 32k
	REGION_VMA (rwx)  : ORIGIN = 0x44444444, LENGTH = 32k
	REGION_LMA (rx)   : ORIGIN = 0x88888888, LENGTH = 32k
}

SECTIONS
{
	kod :
	{
		*(.text*)
	} > REGION_PRVI
	
	podaci :
	{
		podaci_start_vma_2 = .;
		*(.data*)
	} > REGION_VMA AT> REGION_LMA
	
	podaci_start_vma_1 = ADDR(podaci);
	podaci_start_lma = LOADADDR(podaci);
}