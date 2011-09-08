#ifndef __GPIO_H 
#define __GPIO_H

#include <scandal/types.h>

void GPIO_Init(void);
void GPIO_SetInterrupt(uint32_t portNum, uint32_t bitPosi, uint32_t sense,
						uint32_t single, uint32_t event);
void GPIO_IntEnable(uint32_t portNum, uint32_t bitPosi);
void GPIO_IntDisable(uint32_t portNum, uint32_t bitPosi);
uint32_t GPIO_IntStatus(uint32_t portNum, uint32_t bitPosi);
void GPIO_IntClear(uint32_t portNum, uint32_t bitPosi);
void GPIO_SetValue(uint32_t portNum, uint32_t bitPosi, uint32_t bitVal);
void GPIO_ToggleValue(uint32_t portNum, uint32_t bitPosi);
uint32_t GPIO_GetValue(uint32_t portNum, uint32_t bitPosi);
void GPIO_SetDir(uint32_t portNum, uint32_t bitPosi, uint32_t dir);

#endif /* end __GPIO_H */
