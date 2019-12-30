/**
  ******************************************************************************
  * @file    stm32_Uart_HAL.c
  * @author  Thai Pham
  * @version V1.0
  * @date    October 01, 2009
  * @brief   This file provides firmware functions to manage
  *          COM ports available on STM32 from STMicroelectronics.
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
  
/* Includes ------------------------------------------------------------------*/
#include "platform_config.h"
#include "stm32_Uart_HAL.h"
#include "radio.h"

/** @defgroup Private_Variables
  * @{
  */ 
GPIO_TypeDef* GPIO_PORT[FGPIOn] = {RESET_GPIO_PORT, CONFIG_GPIO_PORT,
                                   LED0_GPIO_PORT, LED1_GPIO_PORT, LED2_GPIO_PORT, RTS_GPIO_PORT};
const uint16_t GPIO_PIN[FGPIOn] = {RESET_GPIO_PIN, CONFIG_GPIO_PIN,
                                   LED0_GPIO_PIN, LED1_GPIO_PIN, LED2_GPIO_PIN, RTS_GPIO_PIN};
const uint32_t GPIO_CLK[FGPIOn] = {RESET_GPIO_CLK, CONFIG_GPIO_CLK,
                                   LED0_GPIO_CLK, LED1_GPIO_CLK, LED2_GPIO_CLK, RTS_GPIO_CLK};

USART_TypeDef* COM_USART[COMn] = {USART1, USART2, USART3}; 

GPIO_TypeDef*  COM_PORT[COMn] = {GPIOA, GPIOA, GPIOB};

const uint32_t COM_USART_CLK[COMn] = {RCC_APB2Periph_USART1, RCC_APB1Periph_USART2, RCC_APB1Periph_USART3};

const uint32_t COM_POR_CLK[COMn] = {RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB};

const uint16_t COM_TX_PIN[COMn] = {GPIO_Pin_9, GPIO_Pin_2, GPIO_Pin_10};

const uint16_t COM_RX_PIN[COMn] = {GPIO_Pin_10, GPIO_Pin_3, GPIO_Pin_11};

 
/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM3  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(COM_POR_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    if (COM == COM1)
    {
        RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE); 
    }
    else if ((COM == COM3) || (COM == COM2))
    {
        RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
    }
    else
    {
        // not in config
        Error_Set(COM_TypeDef_WRONG);
    }

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(COM_PORT[COM], &GPIO_InitStructure);
        
    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(COM_PORT[COM], &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(COM_USART[COM], USART_InitStruct);
        
    // /* Enable USART */
    // USART_Cmd(COM_USART[COM], ENABLE);
    
    // /* Configure CTS pin  */
    // STM_CTS_Init();
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void Enable_UART( COM_TypeDef COM )
{
    USART_Cmd(COM_USART[COM], ENABLE);
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void Disable_UART( COM_TypeDef COM )
{
    USART_Cmd(COM_USART[COM], DISABLE);
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
uint8_t UART_Read_Byte( COM_TypeDef COM )
{
unsigned short char_rx;

    char_rx = (unsigned short)USART_ReceiveData(COM_USART[COM]);
    
    if ((USART_GetFlagStatus(COM_USART[COM], USART_FLAG_FE) == SET)||\
    (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_PE) == SET)||\
    (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_ORE) == SET))
    {
        // Framing Error
        Error_Set(UART_Read_Byte_FALSE);
    }
    return ((uint8_t)(char_rx & RX_DATA_MASK));
}
/**
  * @brief  .
  * @param  None
  * @retval None
  */
ITStatus COM_Get_RXIT_Status( COM_TypeDef COM )
{
    return (USART_GetITStatus(COM_USART[COM], USART_IT_RXNE));
}
/**
  * @brief  .
  * @param  None
  * @retval None
  */
ITStatus COM_Get_Error_Status( COM_TypeDef COM )
{
    ITStatus retval;
    if((USART_GetFlagStatus(COM_USART[COM], USART_FLAG_ORE) == SET)||
        (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_NE) == SET)||
        (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_PE) == SET)||
        (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_FE) == SET))
    {
        retval = SET;
    }
    else
    {
        retval = RESET;
    }
    return retval;
}
/**
  * @brief  .
  * @param  None
  * @retval None
  */
ITStatus COM_Get_TXIT_Status( COM_TypeDef COM )
{
    return (USART_GetITStatus(COM_USART[COM], USART_IT_TXE));
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
FlagStatus COM_Get_RXFlag_Status( COM_TypeDef COM )
{
    return (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_RXNE));
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
FlagStatus COM_Get_TX_Complete_Flag_Status( COM_TypeDef COM )
{
    // return (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_RXNE));
    return (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TC));
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void COM_Enable_Tx_Irq( COM_TypeDef COM )	/* enable tx interrupt */
{
    USART_ITConfig(COM_USART[COM], USART_IT_TXE, ENABLE); 
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void COM_Disable_Tx_Irq( COM_TypeDef COM )	/* enable tx interrupt */
{
    USART_ITConfig(COM_USART[COM], USART_IT_TXE, DISABLE); 
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void COM_Send_byte( COM_TypeDef COM, uint16_t byte )
{
    USART_SendData(COM_USART[COM], byte);
    while(COM_Get_TX_Complete_Flag_Status(COM) == RESET);
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void COM_Send_byte_without_wait( COM_TypeDef COM, uint16_t byte )
{
    USART_SendData(COM_USART[COM], byte);
    //while(COM_Get_TX_Complete_Flag_Status(COM) == RESET);
}
/**
  * @brief  .
  * @param  None
  * @retval None
  */
void rf_uart_ctrl_enable_master_int(void)
{
    __asm("    mrs     r0, PRIMASK\n"
          "    cpsie   i\n");   

}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void rf_uart_ctrl_disable_master_int(void)
{
    __asm("    mrs     r0, PRIMASK\n"
          "    cpsid   i\n");

}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void Enable_Uart_Irq( COM_TypeDef COM )
{
    USART_ITConfig(COM_USART[COM], USART_IT_RXNE, ENABLE);
    USART_ITConfig(COM_USART[COM], USART_IT_PE, ENABLE);
    USART_ITConfig(COM_USART[COM], USART_IT_ERR, ENABLE);
}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void Disable_Uart_Irq( COM_TypeDef COM )
{
    USART_ITConfig(COM_USART[COM], USART_IT_RXNE, DISABLE);
}

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void COM_NVIC_Configuration( COM_TypeDef COM )
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* Enable the USARTy Interrupt */
    if (COM == COM1)
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    }
    else if (COM == COM2)
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x08; //0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x08; //0;
        
    }
    else if (COM == COM3)
    {
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x07; //0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x09; //0;
    }
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x08; //0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x08; //0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

/**
  * @brief  Configures Functional GPIO.
  * @param  GPIO_NAME: Specifies the specific GPIO to be configured. 
  *   This parameter can be one of following parameters:
  *
  * @retval None
  */
void STM_Spec_GPIOInit(APP_GPIO_TypeDef gpio)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_APB2PeriphClockCmd(GPIO_CLK[gpio], ENABLE);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_PIN[gpio];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIO_PORT[gpio], &GPIO_InitStructure);
    
    // if( gpio == RESET_PIN || gpio == CONFIG_PIN )
        // GPIO_ResetBits(GPIO_PORT[gpio],GPIO_PIN[gpio]);
}

/**
  * @brief  Configures CTS in manual control with EXTI line.
  * @param  
  *   
  *
  * @retval None
  */
void STM_CTS_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //EXTI_InitTypeDef EXTI_InitStructure;
    //NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable Button GPIO clock */
    RCC_APB2PeriphClockCmd(CTS_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

    /* Configure Button pin as input floating */
    GPIO_InitStructure.GPIO_Pin = CTS_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(CTS_GPIO_PORT, &GPIO_InitStructure);

    /* Connect Button EXTI Line to Button GPIO Pin */
    //GPIO_EXTILineConfig(CTS_GPIO_PORT_SOURCE, CTS_GPIO_PIN_SOURCE);  

    /* Configure Button EXTI line */
    /*EXTI_InitStructure.EXTI_Line = CTS_GPIO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    
    EXTI_Init(&EXTI_InitStructure);     */
    
    /* Enable and set Button EXTI Interrupt to the lowest priority */
    /*NVIC_InitStructure.NVIC_IRQChannel = CTS_GPIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x08;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x08;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStructure); */
}

/**
  * @brief  
  * @param   
  *   
  *
  * @retval None
  */
EXTI_EDGE_TypeDef  Check_EXTI_ISR_status( void )
{
    if(GPIO_ReadInputDataBit( CTS_GPIO_PORT, CTS_GPIO_PIN ) == Bit_RESET)
        return FALLING_EDGE;
    else
        return RAISING_EDGE;
}

/**
  * @brief  
  * @param   
  *   
  *
  * @retval None
  */
void CTS_Clear_IT_pending_Bit( void )
{
    EXTI_ClearITPendingBit( CTS_GPIO_EXTI_LINE );
}

/**
  * @brief  Set Functional GPIO at High level
  * @param  GPIO_NAME: Specifies the specific GPIO to be configured. 
  *   This parameter can be one of following parameters:
  *
  * @retval None
  */
void STM_Spec_GPIO_HIGH(APP_GPIO_TypeDef gpio)
{
    GPIO_SetBits(GPIO_PORT[gpio],GPIO_PIN[gpio]);
}

/**
  * @brief  Configures Functional GPIO.
  * @param  GPIO_NAME: Specifies the specific GPIO to be configured. 
  *   This parameter can be one of following parameters:
  *
  * @retval None
  */
void STM_Spec_GPIO_LOW(APP_GPIO_TypeDef gpio)
{
    GPIO_ResetBits(GPIO_PORT[gpio], GPIO_PIN[gpio]);
}

/**
  * @brief  Configures Functional GPIO.
  * @param  GPIO_NAME: Specifies the specific GPIO to be configured. 
  *   This parameter can be one of following parameters:
  *
  * @retval None
  */
void STM_Spec_GPIO_Toggle(APP_GPIO_TypeDef gpio)
{
    if(GPIO_ReadOutputDataBit(GPIO_PORT[gpio], GPIO_PIN[gpio])) {
        // LED is off, turn on
        GPIO_WriteBit(GPIO_PORT[gpio], GPIO_PIN[gpio], Bit_RESET);
    } else {
        // LED is on, turn off
        GPIO_WriteBit(GPIO_PORT[gpio], GPIO_PIN[gpio], Bit_SET);
    }
}


/******************* (C) COPYRIGHT 2014 Thai.Pham *****END OF FILE****/

