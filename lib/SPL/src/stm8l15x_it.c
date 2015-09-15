#include "stm8l15x_conf.h"
#include "stm8l15x_it.h"

#ifndef TRAP_IRQ 
//TRAP interrupt routine
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
	while (1){};
}
#endif

#ifndef FLASH_IRQ 
//FLASH Interrupt routine.
INTERRUPT_HANDLER(FLASH_IRQHandler,1)
{
	while (1){};
}
#endif

#ifndef DMA1_CHANNEL0_1_IRQ
//DMA1 channel0 and channel1 Interrupt routine.
INTERRUPT_HANDLER(DMA1_CHANNEL0_1_IRQHandler,2)
{
	while (1){};
}
#endif

#ifndef DMA1_CHANNEL2_3_IRQ
//DMA1 channel2 and channel3 Interrupt routine.
INTERRUPT_HANDLER(DMA1_CHANNEL2_3_IRQHandler,3)
{
	while (1){};
}
#endif

#ifndef RTC_CSSLSE_IRQ
//RTC / CSS_LSE Interrupt routine.
INTERRUPT_HANDLER(RTC_CSSLSE_IRQHandler,4)
{
	while (1){};
}
#endif

#ifndef EXTIE_F_PVD_IRQ
//External IT PORTE/F and PVD Interrupt routine.
INTERRUPT_HANDLER(EXTIE_F_PVD_IRQHandler,5)
{
	while (1){};
}
#endif

#ifndef EXTIB_G_IRQ
//External IT PORTB / PORTG Interrupt routine.
INTERRUPT_HANDLER(EXTIB_G_IRQHandler,6)
{
	while (1){};
}
#endif

#ifndef EXTID_H_IRQ
//External IT PORTD /PORTH Interrupt routine.
INTERRUPT_HANDLER(EXTID_H_IRQHandler,7)
{
	while (1){};
}
#endif

#ifndef EXTI0_IRQ 
//External IT PIN0 Interrupt routine.
INTERRUPT_HANDLER(EXTI0_IRQHandler,8)
{
	while (1){};
}
#endif

#ifndef EXTI1_IRQ 
//External IT PIN1 Interrupt routine.
INTERRUPT_HANDLER(EXTI1_IRQHandler,9)
{
	while (1){};
}
#endif

#ifndef EXTI2_IRQ
//External IT PIN2 Interrupt routine.
INTERRUPT_HANDLER(EXTI2_IRQHandler,10)
{
	while (1){};
}
#endif

#ifndef EXTI3_IRQ 
//External IT PIN3 Interrupt routine.
INTERRUPT_HANDLER(EXTI3_IRQHandler,11)
{
	while (1){};
}
#endif

#ifndef EXTI4_IRQ 
//External IT PIN4 Interrupt routine.
INTERRUPT_HANDLER(EXTI4_IRQHandler,12)
{
	while (1){};
}
#endif
 
#ifndef EXTI5_IRQ 
//External IT PIN5 Interrupt routine.
INTERRUPT_HANDLER(EXTI5_IRQHandler,13)
{
	while (1){};
}
#endif

#ifndef EXTI6_IRQ 
//External IT PIN6 Interrupt routine.
INTERRUPT_HANDLER(EXTI6_IRQHandler,14)
{
	while (1){};
}
#endif

#ifndef EXTI7_IRQHandler
//External IT PIN7 Interrupt routine.
INTERRUPT_HANDLER(EXTI7_IRQHandler,15)
{
	while (1){};
}
#endif

#ifndef LCD_AES_IRQ 
//LCD /AES Interrupt routine.
INTERRUPT_HANDLER(LCD_AES_IRQHandler,16)
{
	while (1){};
}
#endif

#ifndef SWITCH_CSS_BREAK_DAC_IRQ 
//CLK switch/CSS/TIM1 break Interrupt routine.
INTERRUPT_HANDLER(SWITCH_CSS_BREAK_DAC_IRQHandler,17)
{
	while (1){};
}
#endif

#ifndef ADC1_COMP_IRQ 
//ADC1/Comparator Interrupt routine.
INTERRUPT_HANDLER(ADC1_COMP_IRQHandler,18)
{
	while (1){};
}
#endif

#ifndef TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQ 
//TIM2 Update/Overflow/Trigger/Break /USART2 TX Interrupt routine.
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,19)
{
	while (1){};
}
#endif

#ifndef TIM2_CC_USART2_RX_IRQ 
//Timer2 Capture/Compare / USART2 RX Interrupt routine.
INTERRUPT_HANDLER(TIM2_CC_USART2_RX_IRQHandler,20)
{
	while (1){};
}
#endif

#ifndef TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQ 
//Timer3 Update/Overflow/Trigger/Break Interrupt routine.
INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler,21)
{
	while (1){};
}
#endif

#ifndef TIM3_CC_USART3_RX_IRQ 
//Timer3 Capture/Compare /USART3 RX Interrupt routine.
INTERRUPT_HANDLER(TIM3_CC_USART3_RX_IRQHandler,22)
{
	while (1){};
}
#endif

#ifndef TIM1_UPD_OVF_TRG_COM_IRQ 
//TIM1 Update/Overflow/Trigger/Commutation Interrupt routine.
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler,23)
{
	while (1){};
}
#endif

#ifndef TIM1_CC_IRQ 
//TIM1 Capture/Compare Interrupt routine.
INTERRUPT_HANDLER(TIM1_CC_IRQHandler,24)
{
	while (1){};
}
#endif

#ifndef TIM4_UPD_OVF_TRG_IRQ
//TIM4 Update/Overflow/Trigger Interrupt routine.
INTERRUPT_HANDLER(TIM4_UPD_OVF_TRG_IRQHandler,25)
{
	while (1){};
}
#endif

#ifndef SPI1_IRQ 
//SPI1 Interrupt routine.
INTERRUPT_HANDLER(SPI1_IRQHandler,26)
{
	while (1){};
}
#endif

#ifndef USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQ 
//USART1 TX / TIM5 Update/Overflow/Trigger/Break Interrupt  routine.
INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler,27)
{
	while (1){};
}
#endif

#ifndef USART1_RX_TIM5_CC_IRQ 
//USART1 RX / Timer5 Capture/Compare Interrupt routine.
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
{
	while (1){};
}
#endif

#ifndef I2C1_SPI2_IRQ 
//I2C1 / SPI2 Interrupt routine.
INTERRUPT_HANDLER(I2C1_SPI2_IRQHandler,29)
{
	while (1){};
}
#endif