/**
  ******************************************************************************
  * @file    Project/STM8L15x_StdPeriph_Template/stm8l15x_conf.h
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    30-September-2014
  * @brief   Library configuration file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8L15x_CONF_H
#define __STM8L15x_CONF_H

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"

/* Uncomment the line below to enable peripheral header file inclusion */
//#include "stm8l15x_adc.h"
//#include "stm8l15x_aes.h"
//#include "stm8l15x_beep.h"
#include "stm8l15x_clk.h"
//#include "stm8l15x_comp.h"
//#include "stm8l15x_dac.h"
//#include "stm8l15x_dma.h"
#include "stm8l15x_exti.h"
//#include "stm8l15x_flash.h"
#include "stm8l15x_gpio.h"
//#include "stm8l15x_i2c.h"
//#include "stm8l15x_irtim.h"
//#include "stm8l15x_itc.h"
//#include "stm8l15x_iwdg.h"
#include "stm8l15x_lcd.h"
//#include "stm8l15x_pwr.h"
//#include "stm8l15x_rst.h"
#include "stm8l15x_rtc.h"
//#include "stm8l15x_spi.h"
//#include "stm8l15x_syscfg.h"
#include "stm8l15x_tim1.h"
//#include "stm8l15x_tim2.h"
//#include "stm8l15x_tim3.h"
//#include "stm8l15x_tim4.h"
//#include "stm8l15x_tim5.h"
//#include "stm8l15x_usart.h"
//#include "stm8l15x_wfe.h"
//#include "stm8l15x_wwdg.h"

/* Uncomment the line below to create your own interrupt handlers */
//#define TRAP_IRQ                            /* TRAP */
//#define FLASH_IRQ                           /* FLASH EOP/PG_DIS */
//#define DMA1_CHANNEL0_1_IRQ                 /* DMA1 Channel0/1 */
//#define DMA1_CHANNEL2_3_IRQ                 /* DMA1 Channel2/3 */
//#define RTC_CSSLSE_IRQ                      /* RTC /CSS_LSE */
//#define EXTIE_F_PVD_IRQ                     /* EXTI PORTE/EXTI PORTF/PVD */
//#define EXTIB_G_IRQ                         /* EXTI PORTB / EXTI PORTG */
//#define EXTID_H_IRQ                         /* EXTI PORTD / EXTI PORTH */
//#define EXTI0_IRQ                           /* EXTI PIN0 */
//#define EXTI1_IRQ                           /* EXTI PIN1 */
#define EXTI2_IRQ                           /* EXTI PIN2 */
#define EXTI3_IRQ                           /* EXTI PIN3 */
//#define EXTI4_IRQ                           /* EXTI PIN4 */
//#define EXTI5_IRQ                           /* EXTI PIN5 */
//#define EXTI6_IRQ                           /* EXTI PIN6 */
//#define EXTI7_IRQ                           /* EXTI PIN7 */
//#define LCD_AES_IRQ                         /* LCD /AES */
//#define SWITCH_CSS_BREAK_DAC_IRQ            /* Switch CLK/CSS/TIM1 Break/DAC */
//#define ADC1_COMP_IRQ                       /* ADC1/COMP*/
//#define TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQ  /* TIM2 UPD/OVF/TRG/BRK / USART2 TX */
//#define TIM2_CC_USART2_RX_IRQ               /* TIM2 CAP / USART2 RX */
//#define TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQ  /* TIM3 UPD/OVF/TRG/BRK /USART3 TX */
//#define TIM3_CC_USART3_RX_IRQ               /* TIM3 CAP/ USART3 RX */
#define TIM1_UPD_OVF_TRG_COM_IRQ            /* TIM1 UPD/OVF/TRG/COM */
//#define TIM1_CC_IRQ                         /* TIM1 CAP */
//#define TIM4_UPD_OVF_TRG_IRQ                /* TIM4 UPD/OVF/TRI */
//#define SPI1_IRQ                            /* SPI1 */
//#define USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQ  /* USART1 TX / TIM5 UPD/OVF/TRG/BRK */
//#define USART1_RX_TIM5_CC_IRQ               /* USART1 RX / TIM5 CAP */
//#define I2C1_SPI2_IRQ                       /* I2C1 / SPI2 */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
   Standard Peripheral Library drivers code */
#define USE_FULL_ASSERT    

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed. 
  *   If expr is true, it returns no value.
  * @retval : None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM8L15x_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
