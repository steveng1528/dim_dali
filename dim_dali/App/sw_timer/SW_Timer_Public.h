/** @file SW_Timer_Public.h
 *  @brief 
 *
 *
 *  @author : Thai Pham
 *  @author :
 *  @bug No known bugs.
 *  @day : March 6th, 2016
 */

 /**
 * @enum SW_TIMER_ENUM
 * @brief enum that list all SW timer
 */
typedef enum
{
	GPRS_CONNECTION = 0, 				
	GPRS_Rx_Timeout,	
	GSM_Delay_Timer,	/*for timeout send message to Server */
	GPRS_REFRESH_OPCODE,
	SMS_DELAY_TIMER,	/*for control SMS task*/
	
	iTOTAL_CONTROL_DIMMING, /*for control itotal*/
	
	LCD_LOCKPAGE_TIME, /*time or lock keypad handle*/
	
	SDCARD_TIMER_DELAY,	/*for sdcard*/
	
	BLUETOOTH_RX_DELAY,	/*for bluetooth*/
	BLUETOOTH_TX_DELAY,
	
	RS485ADAPTER_WAIT_RX_CHAR, /*for RS485 ADAPTER*/
	RS485ADAPTER_WAIT_NEXT_TX, /*for RS485 ADAPTER*/
	RS485_AUTO_UPDATE_DATA,
	
	CONTACTOR_TIEMOUT, /*time for control Contactor - thoi gian contactor dong khong duoc lam gi ca vi rat de bi nhieu loan do contactor xa dien*/
	
	FLASH_Handle_TimeOut,	/*time for Flash handle - flash can thoi gian phuc hoi, khong cho can thiep flash qua nhieu*/
	
	TOTAL_TIMER
} SW_TIMER_ENUM;

#define ROUTER_DELAY_BEFORE_RESET  SINGLE_SEND_TIMEOUT

//! \var ALARM_MYFUNCPTR
//! \brief Is a pointer to the function to do according to the current state, internal event and next event.
typedef void (*SW_TIMER_CALLBACK)(void);

/* interface functions -------------------------------------------------------------*/
extern void SW_Timer_init( void );
extern void SW_Timer_clear(unsigned char Loc_TIMER);
extern void SW_Timer_create( SW_TIMER_ENUM Loc_TIMER, unsigned int timeout_in_mS /*, TimerExpiredCallBack func*/ );
extern void SW_Timer_create_Fcallback( SW_TIMER_ENUM Loc_TIMER, unsigned int timeout_in_mS , SW_TIMER_CALLBACK func );
extern void SW_Timer_start( SW_TIMER_ENUM Loc_TIMER );
extern void SW_Timer_Stop(SW_TIMER_ENUM Loc_TIMER);
extern void SW_Timer_process_ISR( void );
extern unsigned char Get_Timer_expired(SW_TIMER_ENUM Loc_TIMER);
extern unsigned int Get_Timer_Counter(SW_TIMER_ENUM Loc_TIMER);
extern unsigned int Get_Timer_setting(SW_TIMER_ENUM Loc_TIMER);
extern unsigned char Get_Timer_Is_Running(SW_TIMER_ENUM Loc_TIMER);

