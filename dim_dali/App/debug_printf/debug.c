
/**
  ******************************************************************************
  * @file           : debug.c
  * @brief          : this file in implement marcos that is show 
	*                   debug's data by printf function
  ******************************************************************************
  */
	
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <debug.h>


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define DEBUG_PRINTF_UART 												USART1 /*! define uart1 show data to console by printf function*/
#define DEBUG_PRINTF_UART_BRD 										115200 /*! define uart1's baudrate is 115200*/
#define DEBUG_PRINTF_BYTE_COUNT_TX_LOOP						250	/*! times loop for waiting TXE flag is set*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef dbg_huart;

/*Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
	

PUTCHAR_PROTOTYPE
{
	uint8_t u8_TmpCount;
  /*Place your implementation of fputc here */
  /*e.g. write a character to the USART */
	dbg_huart.Instance->DR = (uint8_t)ch;

  /*Loop until the end of transmission */
	for(u8_TmpCount =0; u8_TmpCount < DEBUG_PRINTF_BYTE_COUNT_TX_LOOP; u8_TmpCount ++)
	{
		if(dbg_huart.Instance->SR &0x00000040) 
		{
			/*if TC flag is set, breaking the loop*/
			break;
		}
		else; /*continue loop*/
	}

  return ch;
}


/*Public function -----------------------------------------*/

/**
  * @brief Debug by printf Initialization Function
  * @param None
  * @retval None
  */
uint8_t u8_Debug_Printf_Uart_Init(void)
{
	dbg_huart.Instance = DEBUG_PRINTF_UART;
  dbg_huart.Init.BaudRate = DEBUG_PRINTF_UART_BRD;
  dbg_huart.Init.WordLength = UART_WORDLENGTH_8B;
  dbg_huart.Init.StopBits = UART_STOPBITS_1;
  dbg_huart.Init.Parity = UART_PARITY_NONE;
  dbg_huart.Init.Mode = UART_MODE_TX_RX;
  dbg_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  dbg_huart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&dbg_huart) != HAL_OK)
  {
    Error_Handler();
  }
	return 0;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

