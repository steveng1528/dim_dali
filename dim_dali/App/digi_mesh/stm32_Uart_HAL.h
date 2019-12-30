/**
  ******************************************************************************
  * @file    stm32_Uart_HAL.h
  * @author  Thai Pham
  * @version V1.0
  * @date    October 01, 2009
  * @brief   Header file for stm32_Uart_HAL.c module.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_UART_HAL
#define __STM32_UART_HAL

#ifdef __cplusplus
 extern "C" {
#endif 

/* GLOBAL DEFINE -------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_usart.h"

/**
  * @}
  */
#define COMn                        3
#define FGPIOn                      6
#define     RX_DATA_MASK            0xFF

typedef enum 
{
  COM1 = 0,
  COM2 = 1,
  COM3 = 2
} COM_TypeDef;

typedef enum 
{
  RESET_PIN  = 0,
  CONFIG_PIN = 1,
  LED0_PIN   = 2,
  LED1_PIN   = 3,
  LED2_PIN   = 4,
  RTS_PIN    = 5,
} APP_GPIO_TypeDef;

typedef enum 
{
  FALLING_EDGE  = 0,
  RAISING_EDGE  = 1,
} EXTI_EDGE_TypeDef;
 
/**
  * @}
  */ 
void rf_uart_ctrl_enable_master_int(void);
void rf_uart_ctrl_disable_master_int(void);
void Enable_Uart_Irq( COM_TypeDef COM );
void Disable_Uart_Irq( COM_TypeDef COM );
void STM_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct);
void COM_NVIC_Configuration( COM_TypeDef COM );
void STM_Spec_GPIOInit(APP_GPIO_TypeDef gpio);
void Enable_UART( COM_TypeDef COM );
void Disable_UART( COM_TypeDef COM );
uint8_t UART_Read_Byte( COM_TypeDef COM );
ITStatus COM_Get_RXIT_Status( COM_TypeDef COM );
ITStatus COM_Get_Error_Status( COM_TypeDef COM );
ITStatus COM_Get_TXIT_Status( COM_TypeDef COM );
FlagStatus COM_Get_RXFlag_Status( COM_TypeDef COM );
FlagStatus COM_Get_TX_Complete_Flag_Status( COM_TypeDef COM );
void COM_Enable_Tx_Irq( COM_TypeDef COM );
void COM_Disable_Tx_Irq( COM_TypeDef COM );
void COM_Send_byte( COM_TypeDef COM, uint16_t byte );
void COM_Send_byte_without_wait( COM_TypeDef COM, uint16_t byte );
void STM_Spec_GPIO_HIGH(APP_GPIO_TypeDef gpio);
void STM_Spec_GPIO_LOW(APP_GPIO_TypeDef gpio);
void STM_Spec_GPIO_Toggle(APP_GPIO_TypeDef gpio);
void STM_CTS_Init( void );
EXTI_EDGE_TypeDef  Check_EXTI_ISR_status( void );
void CTS_Clear_IT_pending_Bit( void );

#ifdef __cplusplus
}
#endif


#endif /* __STM32_EVAL_H */
/**
  * @}
  */ 


/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */     

/******************* (C) COPYRIGHT 2014 Thai.Pham *****END OF FILE****/
