#include <arch/system.h>
#include <scandal/system.h>

void system_reset(void) {
	SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);

	/* Ensure completion of memory access */
	asm("dsb");

	/* wait until reset */
	while(1)
		;
}
