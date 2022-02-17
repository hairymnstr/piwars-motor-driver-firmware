#include "stm32g0xx_ll_gpio.h"

#include "address.h"

uint8_t _my_address = 0xffu;

void address_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    _my_address = (1u << ((LL_GPIO_ReadInputPort(GPIOA) >> 1) & 0x0007));
}

uint8_t my_address(void)
{
    return _my_address;
}
