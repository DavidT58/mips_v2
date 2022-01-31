	.section .text.kod
	ldr r0, =podatak
	ldr r0, [r0]
	
	.section .data.podaci
podatak:
	.word .	@ tacka predstavlja trenutnu lokaciju
	.end
	