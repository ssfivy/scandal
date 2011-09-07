#include <arch/gpio.h>

void PIOINT0_IRQHandler(void)
{
}

void PIOINT1_IRQHandler(void)
{
}

void PIOINT2_IRQHandler(void)
{
}

void PIOINT3_IRQHandler(void)
{
}

void GPIOInit( void )
{
}

void GPIOSetInterrupt( uint32_t portNum, uint32_t bitPosi, uint32_t sense,
			uint32_t single, uint32_t event )
{
}

void GPIOIntEnable( uint32_t portNum, uint32_t bitPosi )
{
}

void GPIOIntDisable( uint32_t portNum, uint32_t bitPosi )
{
}

uint32_t GPIOIntStatus( uint32_t portNum, uint32_t bitPosi )
{
	return 0;
}

void GPIOIntClear( uint32_t portNum, uint32_t bitPosi )
{
}

void GPIOSetValue( uint32_t portNum, uint32_t bitPosi, uint32_t bitVal ) {
}

uint32_t GPIOGetValue( uint32_t portNum, uint32_t bitPosi) {
	return 0;
}

void GPIOToggleValue(uint32_t portNum, uint32_t bitPosi) {
}

void GPIOSetDir( uint32_t portNum, uint32_t bitPosi, uint32_t dir )
{
}
