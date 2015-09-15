/** include ----------------------------------------------------------------- */
#include "stm8l15x_conf.h"
#include "68ps_display.h"
#include "bmp180.h"
#include "dht22.h"
#include "delay.h"

/** global variables -------------------------------------------------------- */
int32_t mt;
float mpr;
__IO bool is_pr = FALSE;

/** interrupt handlers ------------------------------------------------------ */
INTERRUPT_HANDLER(EXTI2_IRQHandler,10)
{
  is_pr = FALSE;
  EXTI_ClearITPendingBit(EXTI_IT_Pin2);
}

INTERRUPT_HANDLER(EXTI3_IRQHandler,11)
{
  is_pr = TRUE;
  EXTI_ClearITPendingBit(EXTI_IT_Pin3);
}

INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler,23)
{
  GPIO_ToggleBits(GPIOC, GPIO_Pin_3 | GPIO_Pin_5);
  TIM1_ClearITPendingBit(TIM1_IT_Update);
}

/** functions --------------------------------------------------------------- */
int SystemInit(void)
{
  // clk
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
  // gpio
  GPIO_Init(GPIOC, GPIO_Pin_3 | GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Slow); // leds
  GPIO_Init(GPIOA, GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_In_PU_IT);        // buttons
  EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Falling);
  EXTI_SetPinSensitivity(EXTI_Pin_3, EXTI_Trigger_Falling);
  // timer 1
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, ENABLE);
  TIM1_TimeBaseInit(1599, TIM1_CounterMode_Up, 10000, 0);
  TIM1_ITConfig(TIM1_IT_Update, ENABLE);
  TIM1_Cmd(ENABLE);
  // lcd
  LCD_HardwareInit();
  // bmp180
  BMP180_Init();
  // dht22
  DHT22_Init();
  // interrupts
  enableInterrupts();
  return 0;
}

void main(void)
{
  SystemInit();
  GPIOC->ODR |= GPIO_Pin_5;
  GPIOC->ODR &= ~GPIO_Pin_3;
  
  while (1)
  {
    
    BMP180_StartMeasureTemp();
    Delay(4000);                  // ~8 ms
    mt = BMP180_GetTemp();
    BMP180_StartMeasurePressure(PressureMeasurementMode_ST);
    Delay(50000);                  // ~100 ms
    mpr = BMP180_GetPressureInMMHG();
    if (is_pr)
      LCD_WriteFloat(mpr, 1);
    else
      LCD_WriteFloat(mt/10.0, 1);
    
    /*
    float temp, humid;
    DHT22_GetData(&temp, &humid);
    LCD_WriteFloat(humid, 1);
    Delay(4000000);
    */
    /*
    uint8_t temp11 = 0, humid11 = 0, status;
    status = DHT11_GetData(&temp11, &humid11);
    LCD_WriteInt(humid11);
    Delay(4000000);
    */
    /*
    Delay(1000000);
    DHT_ReadData(dat);
    for (int j = 0; j < 5; j++)
    {
      LCD_WriteInt(j);
      Delay(1000000);
      LCD_WriteInt(dat[j]);
      Delay(2000000);
    }
    */
  };
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {}
}
#endif
