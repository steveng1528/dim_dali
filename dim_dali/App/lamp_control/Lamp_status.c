/**
  ******************************************************************************
  * @file    Lamp_staus.c
  * @author  Thai Pham
  * @version V1.0
  * @date    October 01, 2009
  * @brief   stored and provide Lamp status
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
  
/* declare variable ---------------------------------------------------------*/
#include  "Lamp_status.h"
/* declare variable ---------------------------------------------------------*/
LAMP_STATUS_TYPE    Lamp_status_val;
LAMP_COMMAND_TYPE   Lamp_cmd_signal;


void lamp_set_Name(unsigned char * name)
{
    Lamp_status_val.name[0] = name[0];
    Lamp_status_val.name[1] = name[1];

}
void lamp_set_Voltage(unsigned char * vol)
{
    Lamp_status_val.Voltage[0] = vol[0];
    Lamp_status_val.Voltage[1] = vol[1];

}
void lamp_set_Current(unsigned char * current)
{
    Lamp_status_val.Current[0] = current[0];
    Lamp_status_val.Current[1] = current[1];

}
void lamp_set_Power(unsigned char * Power)
{
    Lamp_status_val.Power[0] = Power[0];
    Lamp_status_val.Power[1] = Power[1];

}
void lamp_set_temperature(unsigned char * temp)
{
    Lamp_status_val.Temperature = temp[0];

}
void lamp_set_DimLevel(unsigned char * dim)
{
    Lamp_status_val.Dim_Level= dim[0];

}
void lamp_set_Lampstatus(unsigned char * status)
{
    Lamp_status_val.lamp_status= status[0];

}

unsigned char lamp_get_command_status( void )
{
    unsigned char retval;
    
    if(0 < Lamp_cmd_signal.Is_signal ){
        Lamp_cmd_signal.Is_signal = 0;
        retval = Lamp_cmd_signal.DIM_level;
        Lamp_cmd_signal.DIM_level = 0xff;
    }else{
        retval = 0xFF;
    }
return retval;
}

