/*************************************************************************
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
		- In the interrupt handler of TIM6 (or any timer selected for dali) add the line below:
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
**************************************************************************/
#ifndef __DALI_H
#define __DALI_H
#endif

#ifdef __cplusplus
 extern "C" {
#endif
//------------------------------------------------------------------------//
#include "gpio.h"
#include "main.h"

//----------------------------------DEFINE--------------------------------//
typedef union{
	uint8_t BYTE;
	struct {
		uint8_t BIT7:1;
		uint8_t BIT6:1;
		uint8_t BIT5:1;
		uint8_t BIT4:1;
		uint8_t BIT3:1;
		uint8_t BIT2:1;
		uint8_t BIT1:1;
		uint8_t BIT0:1;
	}_BIT;
	
}_ONEBYTE;

extern  _ONEBYTE DATA_ONEBYTE;
extern _ONEBYTE DALI_STT;

typedef union{
	
	uint16_t WORD;
	struct {
		uint8_t BIT15:1;	
		uint8_t BIT14:1;
		uint8_t BIT13:1;
		uint8_t BIT12:1;
		uint8_t BIT11:1;
		uint8_t BIT10:1;
		uint8_t BIT9:1;
		uint8_t BIT8:1;
		
		uint8_t BIT7:1;
		uint8_t BIT6:1;
		uint8_t BIT5:1;
		uint8_t BIT4:1;
		uint8_t BIT3:1;
		uint8_t BIT2:1;
		uint8_t BIT1:1;
		uint8_t BIT0:1;
	}_BIT;
	
}_TWOBYTE;

extern  _TWOBYTE DATA_TWOBYTE;
typedef struct {
	
	uint16_t FULLBYTE;
	uint16_t Time_cnt;
	uint16_t CNT1;
	uint16_t CNT2;
	uint16_t CNT3;
	uint16_t CNT4;
	
	uint16_t CNT5;
	uint16_t CNT6;
	uint16_t CNT7;
	uint16_t CNT8;
	uint16_t CNT9;
	uint16_t CNT10;
	uint16_t CNT11;
	uint16_t CNT12;
	uint16_t CNT13;
	uint16_t CNT14;
	uint16_t CNT15;
	uint16_t CNT16;
	
	uint16_t Half_Bit1_1;
	uint16_t Half_Bit1_2;
	uint16_t Half_Bit2_1;
	uint16_t Half_Bit2_2;
	uint16_t Half_Bit3_1;
	uint16_t Half_Bit3_2;
	uint16_t Half_Bit4_1;
	uint16_t Half_Bit4_2;
	uint16_t Half_Bit5_1;
	uint16_t Half_Bit5_2;
	uint16_t Half_Bit6_1;
	uint16_t Half_Bit6_2;
	uint16_t Half_Bit7_1;
	uint16_t Half_Bit7_2;
	uint16_t Half_Bit8_1;
	uint16_t Half_Bit8_2;
	uint16_t Half_Bit9_1;
	uint16_t Half_Bit9_2;
	uint16_t Half_Bit10_1;
	uint16_t Half_Bit10_2;
	uint16_t Half_Bit11_1;
	uint16_t Half_Bit11_2;

}__DALI;
extern __DALI _TIME;



typedef struct {
	uint16_t DALI_STT;	
	uint16_t DALI_CTR_GEAR;
	uint16_t DALI_LAMP_FAIL;
	uint16_t DALI_POWER_ON;
	uint16_t DALI_LIMIT_ERR;
	uint16_t DALI_RST_STT;
	uint16_t DALI_MISS_SHORT_ADD;	
	uint16_t DALI_CT_DTR;	
	uint16_t DALI_VERSION;
	uint16_t DALI_COTENT_DTR;
	uint16_t DALI_DV_TYPE;
	uint16_t PHY_MIN_LEVEL;
	uint16_t PHY_PW_FAIL;
	uint16_t PHY_CT_DRT1;
	uint16_t PHY_CT_DRT2;

	uint16_t DALI_ACTUAL_LV;
	uint16_t DALI_MAX_LV;
	uint16_t DALI_MIN_LV;
	
	uint16_t DALI_POW_ON_LV;
	uint16_t DALI_SYS_FAIL_LV;
	uint16_t DALI_FAD_TIME;

}__DALI_STT;
extern __DALI_STT _DALI_STT;
typedef uint8_t BOOL;

typedef unsigned char UCHAR;
typedef char CHAR;

typedef uint16_t USHORT;
typedef int16_t SHORT;

typedef uint32_t ULONG;
typedef int32_t LONG;

#ifndef TRUE
#define TRUE      1
#endif

#ifndef FALSE
#define FALSE     0
#endif

#ifndef ON
#define ON				1
#endif

#ifndef OFF
#define OFF				0
#endif

// DALI IO
#define DALI_PORT	(GPIOA)
#define DALI_TX_PIN	(GPIO_PIN_9)
#define DALI_RX_PIN	(GPIO_PIN_10)

#define PIN_LOW (GPIO_PIN_RESET)
#define PIN_HIGH (GPIO_PIN_SET)

// DALI ADDRESS INDICATE BITS
#define DALI_ADD_Y_BIT									7
#define DALI_ADD_S_BIT									0

// DALI TIMER
#define DALI_QUATER_BIT									1  //218us
#define DALI_HALF_BIT										2	//416us
#define DALI_FULL_BIT										4

// DALI ADDRESS DEFAULT
#define DALI_ADD_DEF										0				// Default address

// DALI COMMAND
//---OUTPUT LEVEL INSTRUCTIONS---//
#define DALI_CMD_OFF										0				// Turn off lamp
#define DALI_CMD_UP_200MS								1				// Increases lamp(s) illumination level
#define DALI_CMD_DOWN_200MS							2				// Decreases lamp(s) illumination level
#define DALI_CMD_STEP_UP								3				// Increases the target illumination level by 1
#define DALI_CMD_STEP_DOWN							4				// Decreases the target illumination level by 1
#define DALI_CMD_RECALL_MAX							5				// Recall Max – Level 
#define DALI_CMD_RECALL_MIN							6				// Recall Min – Level 

#define DALI_CMD_STEP_DOWN_AND_OFF			7				// If the target level is zero, lamp(s) are turned
																								// off; if the target level is between the min. and
																								// max. levels, decrease the target level by one;
																								// if the target level is max., lamp(s) are turned
																								// off
																								
#define DALI_CMD_ON_AND_STEP_UP					8				// If the target level is zero, lamp(s) are set to
																								// minimum level; if target level is between min.
																								// and max. levels, increase the target level by
																								// one
																								
// The instructions from 16 to 31 -> Go to scene 1-16

#define DALI_CMD_RESET											32			// Configures all variables back to their Reset state
#define DALI_CMD_STORE_ACTUAL_TO_DTR				33			// Store the actual level into DTR
#define DALI_CMD_STORE_DTR_AS_MAX						42			// Store the DTR as Max–Level 
#define DALI_CMD_STORE_DTR_AS_MIN						43			// Store the DTR as Min–Level 
#define DALI_CMD_STORE_DTR_AS_FAIL					44			// Store the DTR as System Failure–Level  
#define DALI_CMD_STORE_DTR_AS_POWER_ON			45			// Store the DTR as Power On–Level  
#define DALI_CMD_STORE_DTR_AS_FADE_TIME			46			// Store the DTR as Fade Time
#define DALI_CMD_STORE_DTR_AS_FADE_RATE			47			// Store the DTR as Fade Rate
#define DALI_CMD_STORE_DTR_AS_SHORT_ADD			128			// Store the DTR as Short Address

//---QUERY INSTRUCTIONS---//
#define DALI_CMD_QUERY_STATUS								144			
#define DALI_CMD_QUERY_BALLAST							145
#define DALI_CMD_QUERY_LAMP_FAIL						146
#define DALI_CMD_QUERY_LAMP_POWER_ON				147
#define DALI_CMD_QUERY_RESET_STATE					149
#define DALI_CMD_QUERY_MISSING_SHORT_ADD		150
#define DALI_CMD_QUERY_VERSION							151
#define DALI_CMD_QUERY_CONTENT_DTR					152
#define DALI_CMD_QUERY_DEVICE_TYPE					153


#define DALI_CMD_QUERY_PHY_MIN_LEVEL					154


#define DALI_CMD_QUERY_POWER_FAIL						155

#define DALI_CMD_QUERY_DTR1						156
#define DALI_CMD_QUERY_DTR2						157


#define DALI_CMD_QUERY_ACTUAL_LEVEL					160
#define DALI_CMD_QUERY_MAX_LEVEL						161
#define DALI_CMD_QUERY_MIN_LEVEL						162
#define DALI_CMD_QUERY_POWER_ON_LEVEL				163
#define DALI_CMD_QUERY_SYSTEM_FAIL_LEVEL		164
#define DALI_CMD_QUERY_FADE_TIME_RATE				165			// Query Fade Time / Fade Rate

#define DALI_CMD_QUERY_SHORT_ADD		269

// Functions prototype

void dali_init(void);

uint16_t dali_read(void);
void dali_write(uint8_t add, uint8_t cmd);
void dali_write_Broadcast(uint8_t cmd);

void dali_write_bit(uint8_t wbit);
void dali_write_stop_bit(void);
void dali_write_broadcast(uint8_t cmd); // Broadcast address

uint8_t dali_timer_init(void);
void dali_gpio_init(void);
void dali_timer_start(uint16_t ntimer_208us);
void dali_timer_interrupt_handler(void);
void dali_timer_disable(void);
void dali_timer_start(uint16_t ntimer_208us);
void dali_timer_interrupt_handler(void);
void DALI_read_All_Parmeter(void);
uint8_t DALI_DIM(uint8_t Level);
void DALI_ON_OFF_LAMP_MAX(uint8_t Flag);
void DALI_read_All_Parmeter(void);
void DALI_Lamp_Blink(uint8_t BLinkNumber);



