/********************************************************************************
  * File Name          : dali.c
  * Description        : This file provides code for interfacing with devices 
												 that support DALI
	------------------------------------------------------------------------------	
	Support 2byte communication for ballasts / actuators (64 short address)
	Baud rate is 1200 -> Bit-timming is 1/1200 = 833 (us)											 
	Forward frame: 	[1 bits start][8 bits address][8 bits data][2 bits stop]	
									[1][YAAA AAAS][DDDD DDDD][SS]
  Backward frame: [1 bits start][8 bits data][2 bits stop]
									[1][DDDD DDDD][SS]
									
	Where: 	Start bit = level Hight (logical HIGH )
					Stop bit = Idle line
					--------------------
					Y: Address type bit
						Y = 0: Indicates an individual or short address 
						Y = 1: Indicates a group address or broadcast
					A: Address bits
					S: Selector bit
						S = 0: Data bytes indicate Direct Arc Power level
						S = 1: Data bytes indicate a command
						
	-----------------------------------------------------------------------------
	HOW TO USE IT?
	-----------------------------------------------------------------------------
	Step1: add path
		- Add the path of files dali.c & dali.h into project
		
	Step2: In stm32f1xx_it.c file
		- Add the line below: 
				extern void dali_timer_interrupt_handler(void);
		-	In the interrupt handler of TIM6 (or any timer selected for dali) add the line below:
				dali_timer_interrupt_handler();
	
	Step3: In main.c file
		- Add "dali.h" ex: #include "dali.h"
		- Add function "dali_init();" (before and outside of while(1)) for initialization of dali connection peripheral
		- Inside while(1):
			+ Use dali_write(address, command) to send a command to a gear/actuator (single device) on the network
			+ Use dali_write_broadcast(command) to send a command to all gear/actuator (all of device) on the network
			+ Use dali_read(address) to get data from a gear/actuator (a single device) on the network
	Noted: 	
	- Time out for send a command / get data: 10.5 ms
	- System cycle time should be >= 10.5ms (recommend: 20ms)
		
  *******************************************************************************/
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"
#include "dali.h"
#include <stdio.h> 
#include <stdlib.h> 

//extern __DALI _TIME;

//----------------------------------VARIABLE---------------------------------------
extern TIM_HandleTypeDef htim6;
extern uint8_t Read_PIN;
volatile uint16_t dali_timeout = 0;
volatile uint8_t dali_timer_tick = OFF;
uint16_t dali_timeout_repeat = 0;

//--------------------------------------------------------------------------------
/**
	* @brief Timer configuration with 208us and count up, auto preload, 208ms  = 1/4 time to tranfer/received one bit of dali protocol
	* @param 
	* @reval 	TRUE->1  Configuration Success
	*					
*/
uint8_t dali_timer_init( void )
{
	
	TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = (HAL_RCC_GetPCLK1Freq() / 1000000) - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 208;	// 1/2 bit timming
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  return TRUE;
}

/**
	* @brief Active timer 6 
	* @param ntimer_208us, number repeat timer 208us, ex, if ntimer_208us=20, inveral time = 20*208
	* @reval None
	*					
*/
 
//-------------------------------------------------------------------------------- 
void dali_timer_start(uint16_t ntimer_208us)
{
	dali_timeout = ntimer_208us; 
	dali_timeout_repeat = ntimer_208us;
  HAL_TIM_Base_Start_IT(&htim6);
}

/**
	* @brief Stop timer 6 
	* @param None
	* @reval None
	*					
*/

//-------------------------------------------------------------------------------- 
void dali_timer_disable(void)
{
  /* Disable any pending timers. */
  HAL_TIM_Base_Stop_IT(&htim6);
}

/**
	* @brief Interrupt timer 6
	* @param None
	* @reval None
	*					
*/
//--------------------------------------------------------------------------------
void dali_timer_interrupt_handler(void)
{
	if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET && __HAL_TIM_GET_IT_SOURCE(&htim6, TIM_IT_UPDATE) !=RESET) 
	{
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
		//
		
		_TIME.Time_cnt = dali_timeout;
		if(dali_timeout > 0) 
			dali_timeout--;
		else
		{
			dali_timer_tick = ON; //1  
			dali_timeout = dali_timeout_repeat;			
		}				
  }	
}

//--------------------------------------------------------------------------------
/**
	* @brief Configuration TX,RX pin for DALI communication
	* @param None
	* @reval None
	*					
*/
void dali_gpio_init( void )
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	// TX
	GPIO_InitTypeDef gpioStruct_tx;
  gpioStruct_tx.Pin = DALI_TX_PIN;
  gpioStruct_tx.Mode = GPIO_MODE_OUTPUT_PP;
  gpioStruct_tx.Pull = GPIO_NOPULL;
  gpioStruct_tx.Speed = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(DALI_PORT, &gpioStruct_tx);
  HAL_GPIO_WritePin(DALI_PORT, DALI_TX_PIN, PIN_HIGH);
	
	// RX
	GPIO_InitTypeDef gpioStruct_rx;
  gpioStruct_rx.Pin = DALI_RX_PIN;
  gpioStruct_rx.Mode = GPIO_MODE_INPUT;
  gpioStruct_rx.Pull = GPIO_PULLDOWN;
  gpioStruct_rx.Speed = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(DALI_PORT, &gpioStruct_rx);
  
}	

/**
	* @brief Init configuration timer 6, GPIO Pin
	* @param None
	* @reval None
	*					
*/
//--------------------------------------------------------------------------------
void dali_init( void )
{	
	dali_timer_init();
	dali_gpio_init();
	
	dali_timer_disable();
}

/**
	* @brief Drive 0ne bit to TX DALI pin, 0.5bit  = 4.16us
	* @param wbit: logic low or high
	* @reval None
	*					
*/
//--------------------------------------------------------------------------------
void dali_write_bit(uint8_t wbit)
{
	dali_timer_tick = OFF;
	dali_timer_start(DALI_HALF_BIT);
	// The first half-bit
	if(wbit)
	{	
		if(HAL_GPIO_ReadPin(DALI_PORT, DALI_TX_PIN))//1
			HAL_GPIO_WritePin(DALI_PORT, DALI_TX_PIN, PIN_LOW);
	}	
	else
	{	
		if(!HAL_GPIO_ReadPin(DALI_PORT, DALI_TX_PIN))//0	
			HAL_GPIO_WritePin(DALI_PORT, DALI_TX_PIN, PIN_HIGH);
	}
	while(!dali_timer_tick);
	dali_timer_tick = OFF;
	
	dali_timer_start(DALI_HALF_BIT);// Wait for valid time
	
	// The last half-bit
	if(wbit)
	{	
		if(!HAL_GPIO_ReadPin(DALI_PORT, DALI_TX_PIN))//0
			HAL_GPIO_WritePin(DALI_PORT, DALI_TX_PIN, PIN_HIGH);
	}	
	else
	{	
		if(HAL_GPIO_ReadPin(DALI_PORT, DALI_TX_PIN))//1
			HAL_GPIO_WritePin(DALI_PORT, DALI_TX_PIN, PIN_LOW);
	}
	//
	while(!dali_timer_tick);// Wait for valid time
	dali_timer_disable();
}	

/**
	* @brief Write logic 1 to TX DALI pin with time 4.16us*4 us
	* @param None
	* @reval None
	*					
*/
void dali_write_stop_bit( void )
{
	dali_timer_tick = OFF;
	dali_timer_start(DALI_HALF_BIT * 4);			// 2 bit-timming
	// The first half-bit	
	if(!HAL_GPIO_ReadPin(DALI_PORT, DALI_TX_PIN))
		HAL_GPIO_WritePin(DALI_PORT, DALI_TX_PIN, PIN_HIGH);
	//
	while(!dali_timer_tick);
	dali_timer_disable();
}	

/**
	* @brief Write data to to DALI Gear with ADD
	* @param add: Adress of DALI gear
	* @param cmd: cmd send to DALI gear to quere data
	* @reval None
	*					
*/
void dali_write(uint8_t add, uint8_t cmd)
{
	UCHAR i=0;
	UCHAR wadd = 0;
	// Write Start-bit
	dali_write_bit(1);
	// Write Address
	wadd = (add << 1)| 0x01;
	for(i=0; i<=7; i++)
	{
		dali_write_bit((wadd>>(7-i)) & 0x01);
	}
	// Write Command
	for(i=0; i<=7; i++)
	{
		dali_write_bit((cmd>>(7-i)) & 0x01);
	}
	// Write Stop-bits
	dali_write_stop_bit();
}	

/**
	* @brief Write data all device connect to 
	* cmd: cmd send to DALI gear to quere data
	* @param add: Adress of DALI gear
	* @param cmd: cmd send to DALI gear to quere data
	* @reval None
	*					
*/
void dali_write_broadcast(uint8_t cmd) // Broadcast address
{
	UCHAR i=0;
	UCHAR wadd = 0xFF;
	HAL_Delay(100);
	// Write Start-bit
	dali_write_bit(1);
	// Write Address
	for(i=0; i<=7; i++)
	{
		dali_write_bit((wadd>>(7-i)) & 0x01);
	}
	// Write Command
	for(i=0; i<=7; i++)
	{
		dali_write_bit((cmd>>(7-i)) & 0x01);
	}
	// Write Stop-bits
	dali_write_stop_bit();
}	




/**
	* @brief Read one bit in back frame 
	* @param None
	* @reval BIT
	*					
*/

UCHAR dali_read_bit(void)
{	
	uint8_t BIT_GPIO=0;

	//--------------------------------------//
	dali_timer_tick = OFF;// 1/4 BIT
	dali_timer_start(DALI_QUATER_BIT);
	while(!dali_timer_tick);//dali_timer_tick=1 when enable timer 6
	dali_timer_disable();
	
	//--------------------------------------//
	dali_timer_tick = OFF;// 1/2 BIT
	dali_timer_start(DALI_QUATER_BIT);
	
	while(!dali_timer_tick);//dali_timer_tick=1 when enable timer 6
	dali_timer_disable();
	
	//--------------------------------------//
	dali_timer_tick = OFF;// 3/4 BIT
	dali_timer_start(DALI_QUATER_BIT);
	while(!dali_timer_tick);//dali_timer_tick=1 when enable timer 6
	dali_timer_disable();
	BIT_GPIO = HAL_GPIO_ReadPin(DALI_PORT, DALI_RX_PIN);	
	//--------------------------------------//
	dali_timer_tick = OFF;// 1 BIT
	dali_timer_start(DALI_QUATER_BIT);
	
	while(!dali_timer_tick);//dali_timer_tick=1 when enable timer 6
	dali_timer_disable();	
	
	//--------------------------------------//
	

	return BIT_GPIO;		
}


/**
	* @brief Read one byte in back frame 
	* @param None
	* @reval BYTE back ward
	*					
*/
uint16_t dali_read( void )
{
	UCHAR i=0;
	uint16_t dat = 0;
	// Wait data
	dali_timer_tick = OFF;
	
	dali_timer_start(DALI_HALF_BIT * 40);	// waif for data income 40*208*2 = 16.64ms
	while((!dali_timer_tick) && (HAL_GPIO_ReadPin(DALI_PORT, DALI_RX_PIN)));//1
	dali_timer_disable();
	//
	for(i=0; i<=8; i++)
	{

		if(i==0){ // Remove first bit, this is start bit in backward frame
			_TIME.FULLBYTE=0;
			_TIME.FULLBYTE = dali_read_bit() ;
		} 
		else
			dat|= dali_read_bit() <<(8-i);			
	}
	DATA_ONEBYTE.BYTE = ~dat;
	return  dat;
}	

//--------------------------------------------------------------------------------

void DALI_read_All_Parmeter(void){
			//Query min level
			_DALI_STT.DALI_MIN_LV=0;
			dali_write_broadcast(DALI_CMD_QUERY_MIN_LEVEL);
			_DALI_STT.DALI_MIN_LV = (dali_read());
			
			//Query version
			_DALI_STT.DALI_VERSION=0;
			dali_write_broadcast(DALI_CMD_QUERY_VERSION);	
			_DALI_STT.DALI_VERSION = (dali_read());		
			//Query short address
			_DALI_STT.DALI_MISS_SHORT_ADD=0;
			dali_write_broadcast(DALI_CMD_QUERY_MISSING_SHORT_ADD);	
			_DALI_STT.DALI_MISS_SHORT_ADD = (dali_read());		
			//Quey lamp fail
			_DALI_STT.DALI_LAMP_FAIL=0;	
			dali_write_broadcast(DALI_CMD_QUERY_LAMP_FAIL);	
			_DALI_STT.DALI_LAMP_FAIL = (dali_read());		
			
			//
			_DALI_STT.DALI_CTR_GEAR=0;	
			dali_write_broadcast(DALI_CMD_QUERY_BALLAST);	
			_DALI_STT.DALI_CTR_GEAR = (dali_read());						
			//
			_DALI_STT.DALI_POWER_ON=0;	
			dali_write_broadcast(DALI_CMD_QUERY_LAMP_POWER_ON);	
			_DALI_STT.DALI_POWER_ON = (dali_read());					
			//
			_DALI_STT.DALI_RST_STT=0;	
			dali_write_broadcast(DALI_CMD_QUERY_RESET_STATE);	
			_DALI_STT.DALI_RST_STT = (dali_read());
			//
			_DALI_STT.DALI_CT_DTR=0;	
			dali_write_broadcast(DALI_CMD_QUERY_CONTENT_DTR);	
			_DALI_STT.DALI_CT_DTR = (dali_read());
			//				
			_DALI_STT.DALI_DV_TYPE=0;	
			dali_write_broadcast(DALI_CMD_QUERY_DEVICE_TYPE);	
			_DALI_STT.DALI_DV_TYPE = (dali_read());				
			//
			_DALI_STT.PHY_PW_FAIL=0;	
			dali_write_broadcast(DALI_CMD_QUERY_LAMP_FAIL);	
			_DALI_STT.PHY_PW_FAIL = (dali_read());
			//
			_DALI_STT.DALI_FAD_TIME=0;	
			dali_write_broadcast(DALI_CMD_QUERY_FADE_TIME_RATE);	
			_DALI_STT.DALI_FAD_TIME = (dali_read());				
			//
			_DALI_STT.PHY_CT_DRT1=0;	
			dali_write_broadcast(DALI_CMD_QUERY_DTR1);	
			_DALI_STT.PHY_CT_DRT1 = (dali_read());	
			//
			_DALI_STT.PHY_CT_DRT2=0;	
			dali_write_broadcast(DALI_CMD_QUERY_DTR2);	
			_DALI_STT.PHY_CT_DRT2 = (dali_read());	
			//
			_DALI_STT.PHY_MIN_LEVEL=0;	
			dali_write_broadcast(DALI_CMD_QUERY_DTR2);	
			_DALI_STT.PHY_MIN_LEVEL = (dali_read());				
			//
			_DALI_STT.PHY_PW_FAIL=0;	
			dali_write_broadcast(DALI_CMD_QUERY_POWER_FAIL);	
			_DALI_STT.PHY_PW_FAIL = (dali_read());		
			//
			_DALI_STT.DALI_POW_ON_LV=0;	
			dali_write_broadcast(DALI_CMD_QUERY_POWER_ON_LEVEL);	
			_DALI_STT.DALI_POW_ON_LV = (dali_read());	
			
			
			_DALI_STT.DALI_STT=0;	
			dali_write_broadcast(DALI_CMD_QUERY_STATUS);	
			_DALI_STT.DALI_STT = (dali_read());		
			DALI_STT.BYTE = 	_DALI_STT.DALI_STT;
}



	
/**
	* @brief ON/OFF lamp that connect to DALI Gear
	* @param FLAG = ON:  Enable lamp on
	* @param FLAG = OFF: Enable lamp off
	* @reval NONE
	*					
*/

void DALI_ON_OFF_LAMP_MAX(uint8_t Flag){
	if(Flag==ON){
		dali_write_broadcast(DALI_CMD_RECALL_MAX);
	}else if(Flag==OFF)
	dali_write_broadcast(DALI_CMD_OFF);
}

/**
	* @brief Dimming lamp with one level, write to all device
	* @param Level: 0-254
	* @reval NONE
	*					
*/

uint8_t DALI_DIM(uint8_t Level){
	uint16_t Current_LV_Dim=0;
	uint8_t cnt1=0, cnt2=0;
	Current_LV_Dim=0;
	//_DALI_STT.DALI_ACTUAL_LV=0;	
	
	READ_LEVEL_START:
	dali_write_broadcast(DALI_CMD_QUERY_ACTUAL_LEVEL);
	Current_LV_Dim =dali_read();
	_DALI_STT.DALI_ACTUAL_LV = Current_LV_Dim ;
	
	if(Current_LV_Dim!=Level) {
		cnt2++;
		if(cnt2<=5){
			goto READ_LEVEL_START;
		}
	}	
	
	if(Current_LV_Dim==Level) {
		return Current_LV_Dim;
	}
		
	
	dali_write_broadcast(DALI_CMD_RECALL_MAX);
	GET_LEVEL:
	dali_write_broadcast(DALI_CMD_QUERY_ACTUAL_LEVEL);
	Current_LV_Dim=0;
	_DALI_STT.DALI_ACTUAL_LV=0;
	Current_LV_Dim =dali_read();
	_DALI_STT.DALI_ACTUAL_LV = Current_LV_Dim ;
	if(Current_LV_Dim==0){
		cnt1++;
		if(cnt1<=5){
			goto GET_LEVEL;
		}
	} 
	if(Current_LV_Dim!=Level){
		if(Current_LV_Dim>Level){
			dali_write_broadcast(DALI_CMD_STEP_DOWN_AND_OFF);// dimming led
			goto GET_LEVEL;
		}else if(Current_LV_Dim<Level){
			dali_write_broadcast(DALI_CMD_RECALL_MAX);
			goto GET_LEVEL;
		}
	} 
	return Current_LV_Dim;
}


/**
	* @brief Blink Led lamp
	* @param BLinkNumber: 0-254
	* @reval NONE
	*					
*/
void DALI_Lamp_Blink(uint8_t BLinkNumber){
	for(int i=0; i< BLinkNumber;i++){
			DALI_ON_OFF_LAMP_MAX(OFF);
			HAL_Delay(1000);
			DALI_ON_OFF_LAMP_MAX(ON);
			HAL_Delay(1000);		
	}		

}






