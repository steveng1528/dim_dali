/** @file SW_Timer_Public.c
 *  @brief implement all thing relate to SW Timer.
 *
 *
 *  @author : Thai Pham
 *  @author :
 *  @bug No known bugs.
 *  @day : March 6th, 2016
 */

/* Includes --------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "SW_Timer_Public.h"

/**
 * @struct InputHdl_Handle_t
 * @brief struct for manage SW timer.
 */
typedef struct
{
    unsigned int   countdown;
    unsigned int   time_setting;
    //TimerExpiredCallBack Callback_func;
    unsigned char   Is_Runnning  :1;
    unsigned char   Is_expired :1;
    SW_TIMER_CALLBACK pFunc;
} SW_TIMER_TYPE;
/* Global variable -------------------------------------------------------------*/
SW_TIMER_TYPE  SWTimer_Val[TOTAL_TIMER];
/* Global definitions  ---------------------------------------------------------*/

/* local functions -------------------------------------------------------------*/
void SW_Timer_clear(unsigned char Loc_TIMER);

/**
  * @brief  init SW timer that is called by main process
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
void SW_Timer_init( void )
{
unsigned char i;

	for(i=0; i<TOTAL_TIMER; i++)
	{
		SW_Timer_clear(i);
	}
}
/**
  * @brief  clear timer all
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
void SW_Timer_clear(unsigned char Loc_TIMER)
{
	SWTimer_Val[Loc_TIMER].Is_Runnning    = 0;
	SWTimer_Val[Loc_TIMER].countdown     	= 0xffffffff;
	SWTimer_Val[Loc_TIMER].time_setting  	= 0;
	SWTimer_Val[Loc_TIMER].Is_expired    	= 0;
    SWTimer_Val[Loc_TIMER].pFunc         	= NULL;
}	
/**
  * @brief  call by main process
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
void SW_Timer_main()
{
/*
unsigned char i;
 for(i=0; i<TOTAL_TIMER; i++)
 {
    	if( 1 ==  SWTimer_Val[i].Is_Runnning )
    	{
        	if( 0 == SWTimer_Val[i].countdown )
        	{
            	SWTimer_Val[Loc_TIMER].Callback_func();
        	}
    	}
	}
*/
}
/**
  * @brief  the 1st step to process SW Timer, assign value timer
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
void SW_Timer_create( SW_TIMER_ENUM Loc_TIMER, unsigned int timeout_in_S /*, TimerExpiredCallBack func*/ )
{
	//	SWTimer_Val[Loc_TIMER].time_setting = (timeout_in_S * 1000);
	SWTimer_Val[Loc_TIMER].time_setting = timeout_in_S;
	//	SWTimer_Val[Loc_TIMER].Callback_func = func;
	SWTimer_Val[Loc_TIMER].Is_Runnning = 0;
	SWTimer_Val[Loc_TIMER].Is_expired  = 0;
}

/**
  * @brief  the 1st step to process SW Timer, assign value timer
  *
  * @param  None
  *
  * @retval None
  * @author : Thai Pham
  */
void SW_Timer_create_Fcallback( SW_TIMER_ENUM Loc_TIMER, unsigned int timeout_in_mS , SW_TIMER_CALLBACK func )
{
    //	SWTimer_Val[Loc_TIMER].time_setting = (timeout_in_S * 1000);
    SWTimer_Val[Loc_TIMER].time_setting = timeout_in_mS;
    //	SWTimer_Val[Loc_TIMER].Callback_func = func;
    SWTimer_Val[Loc_TIMER].Is_Runnning = 0;
    SWTimer_Val[Loc_TIMER].Is_expired  = 0;
    SWTimer_Val[Loc_TIMER].pFunc  = func;
}

/**
  * @brief  2nd step that make timer run, then wait to it's expired
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
void SW_Timer_start( SW_TIMER_ENUM Loc_TIMER )
{
    SWTimer_Val[Loc_TIMER].Is_Runnning = 1;
    SWTimer_Val[Loc_TIMER].countdown = SWTimer_Val[Loc_TIMER].time_setting;
    SWTimer_Val[Loc_TIMER].Is_expired = 0;
}
/**
  * @brief  stop timer
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
void SW_Timer_Stop(SW_TIMER_ENUM Loc_TIMER)
{
    SWTimer_Val[Loc_TIMER].Is_Runnning = 0;
    SWTimer_Val[Loc_TIMER].countdown = 0xffffffff;
}
/**
  * @brief  call by system tick ISR that signal every 1ms
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
void SW_Timer_process_ISR( void )
{
unsigned char i;
    for(i=0; i<TOTAL_TIMER; i++)
    {
        if(1 == SWTimer_Val[i].Is_Runnning)
        {
        	if((0 < SWTimer_Val[i].countdown) && (0xffffffff != SWTimer_Val[i].countdown))
        	{
            	SWTimer_Val[i].countdown--;
            	if( 0 == SWTimer_Val[i].countdown )
            	{
                	SWTimer_Val[i].Is_expired = 1;
                	SWTimer_Val[i].countdown = 0xffffffff;
                    SWTimer_Val[i].Is_Runnning = 0;
                    //Call callback function for whom register
                    if(SWTimer_Val[i].pFunc != NULL){
                        SWTimer_Val[i].pFunc();
                    }
            	}
        	}
        }
    }
}

/**
  * @brief  if timer start and then expied after that, functions return to '1'
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
unsigned char Get_Timer_expired(SW_TIMER_ENUM Loc_TIMER)
{
	unsigned char retval;
	if(1 == SWTimer_Val[Loc_TIMER].Is_expired)
	{
		retval = 1;
		SWTimer_Val[Loc_TIMER].Is_expired = 0;
	}
	else
	{
		retval = 0;
	}    
	return retval;
}

/**
  * @brief  it's called in initialize phase that starting system
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
unsigned int Get_Timer_Counter(SW_TIMER_ENUM Loc_TIMER)
{
   return (SWTimer_Val[Loc_TIMER].time_setting - SWTimer_Val[Loc_TIMER].countdown);
}

/**
  * @brief  it's called in initialize phase that starting system
  *         
  * @param  None
  *         
  * @retval None
  * @author : Thai Pham
  */
unsigned char Get_Timer_Is_Running(SW_TIMER_ENUM Loc_TIMER)
{
   return (SWTimer_Val[Loc_TIMER].Is_Runnning);
}

/*---------------------------------------------------------------------------
 *    	 Get_Timer_setting()
 *---------------------------------------------------------------------------
 *
 * Synopsis:  if timer .
 *
 * Return: void
 */
unsigned int Get_Timer_setting(SW_TIMER_ENUM Loc_TIMER)
{
   return (SWTimer_Val[Loc_TIMER].time_setting);
}

