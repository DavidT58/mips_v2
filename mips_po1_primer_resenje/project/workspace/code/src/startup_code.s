.cpu cortex-m3
.fpu softvfp
.syntax unified
.thumb

.extern _main_stack_pointer_value

.extern main

.extern _lma_data_start
.extern _vma_data_start
.extern _vma_data_end

.section .text.reset_handler
.type reset_handler, %function
reset_handler:
	ldr r0, =_lma_data_start
	ldr r1, =_vma_data_start
	ldr r2, =_vma_data_end

	cmp r1, r2
	beq branch_to_main
copy_loop:
	ldr r3, [r0], #4
	str r3, [r1], #4
	cmp r1, r2
	blo copy_loop
branch_to_main:
	bl main
infinite_loop_1:
	b infinite_loop_1

.section .text.default_handler
.type default_handler, %function
default_handler:
infinite_loop_2:
	b infinite_loop_2

.section .vector_table, "a"
.word _main_stack_pointer_value
.word reset_handler
.word nmi
.word hard_fault
.word mem_manage_fault
.word bus_fault
.word usage_fault
.word default_handler
.word default_handler
.word default_handler
.word default_handler
.word sv_call
.word default_handler
.word default_handler
.word pend_sv
.word systick
.word irq0_WWDG
.word irq1_PVD
.word irq2_TAMPER
.rept 65
	.word default_handler
.endr

.weak nmi
.thumb_set nmi, default_handler

.weak hard_fault
.thumb_set hard_fault, default_handler

.weak mem_manage_fault
.thumb_set mem_manage_fault, default_handler

.weak bus_fault
.thumb_set bus_fault, default_handler

.weak usage_fault
.thumb_set usage_fault, default_handler

.weak sv_call
.thumb_set sv_call, default_handler

.weak pend_sv
.thumb_set pend_sv, default_handler

.weak systick
.thumb_set systick, default_handler

.weak irq0_WWDG
.thumb_set irq0_WWDG, default_handler

.weak irq1_PVD
.thumb_set irq1_PVD, default_handler

.weak irq2_TAMPER
.thumb_set irq2_TAMPER, default_handler

.end
