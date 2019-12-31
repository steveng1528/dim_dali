/**
  ******************************************************************************
  * @file    Lamp_staus.h
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

/* typedef definition ---------------------------------------------------------*/
typedef struct{
    unsigned char name[2];
    unsigned char Voltage[2];
    unsigned char Current[2];
    unsigned char Power[2];
    unsigned char Temperature;
    unsigned char Dim_Level;
    unsigned char lamp_status;

}LAMP_STATUS_TYPE;
    
typedef struct{
    unsigned char Is_signal;
    unsigned char DIM_level;

}LAMP_COMMAND_TYPE;
/* function prototype ---------------------------------------------------------*/
void lamp_set_Name(unsigned char * name);
void lamp_set_Voltage(unsigned char * vol);
void lamp_set_Current(unsigned char * current);
void lamp_set_Power(unsigned char * Power);
void lamp_set_temperature(unsigned char * temp);
void lamp_set_DimLevel(unsigned char * dim);
void lamp_set_Lampstatus(unsigned char * status);

unsigned char lamp_get_command_status( void );  // > 100: no command; 0: OFF; 100: ON




