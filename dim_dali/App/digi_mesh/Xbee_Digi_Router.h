/**
  ******************************************************************************
  * @file    Xbee_Digi_Router.h
  * @author  Thai Pham
  * @version V1.0
  * @date    October 01, 2009
  * @brief   Main program body
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

#ifndef  XBEE_DIGI_ROUTER_H
#define  XBEE_DIGI_ROUTER_H

#include "stm32f1xx_hal.h"
#include "Xbee_Router_ATcmd.h"

#define Active_COM      COM3
/* Private typedef ------------------------------------------------------------*/
#define NUM_OF_ROUTER           				110
#define NUMBER_ROUTER_MSG_BYTE  				16  //fix length of msg received from Router

#define GW_READ_STATUS      						0x81
#define GW_READ_SCHEDULE    						0x82
#define GW_SET_SCHEDULE     						0x83
#define GW_SET_NAME         						0x80
#define GW_SET_DIM          						0x84
#define GW_SET_NORMINAL_VALUE   				0x85
#define GW_SET_CONFIG_RF_MODULE 				0x86
#define GW_SET_TIME_REPLY       				0x87
#define GW_RESET_RF_MODULE      				0x88

#define Router_STATUS_REPLY             0x01
#define Router_SCHEDULE_REPLY           0x02
#define Router_REPLY_FROM_SINGLE_CMD    0x03

typedef enum xbee_app_global_state{
    XBEE_G_GET_MODULE_CONFIG = 0,
    XBEE_G_GW_DISCOVERY,
    XBEE_G_GW_CONNECTED
}xbee_app_global_state;

typedef enum{
    XBEE_RX_STATE_WAITSTART = 0,		///< waiting for initial 0x7E
    XBEE_RX_STATE_LENGTH_MSB,				///< waiting for MSB of length (first byte)
    XBEE_RX_STATE_LENGTH_LSB,				///< waiting for LSB of length (second byte)
    XBEE_RX_STATE_RXFRAME,					///< receiving frame and/or trailing checksum
    XBEE_RX_STATE_CHECKSUM
}xbee_dev_rx_state ;

typedef struct {
		xbee_dev_rx_state			state;																/// current state of receiving a frame
    uint16_t							bytes_in_frame;												/// bytes in frame being read; does not include checksum byte
    uint16_t							bytes_read;														/// bytes read so far
    uint8_t								frame_data[XBEE_MAX_FRAME_LEN + 1]; 	/// bytes received, starting with frame_type, +1 is for checksum
} _XBEE_RX;

typedef struct{
    unsigned int  time_delay_In_ms;
    xbee_app_global_state state;
    uint8_t node_ID[2];
    uint8_t frame_ID;
}_XBEE_DEV;

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
    START_MSG,
    COMMAND,
    BRANCH_INDEX,
    LAMP_INDEX,
    RF_DATA,
    SET_DIM_CMD,
    SET_CONFIG_RF_MODULE_CMD,
    SET_TIMEREPLY_MODE,
    /*
    PING_BROADCAST,
    PING_SPEC_NODE,
    CONFIG_RF_MODULE_ROUTER,
    CONFIG_GATEWAY,
    CONFIG_MB_ROUTER,
    ROUTER_RANDOM_SENT,
    GW_SEND_WITH_SYNCOUNTER,
    */
} _RF_RX_STATUS;

typedef struct{
    // unsigned char Host_Rx_Packet[MAX_HOST_READ_PACKET][MAX_HOST_READ_BUFF];
    _RF_RX_STATUS RF_Rx_Status;           //position to write data to buffer
    unsigned char cmd_buf[20];           //position to write data to buffer
    unsigned char cmd_buf_index;           //position to write data to buffer
    unsigned char cmd_type;
    unsigned char cmd_Length;
    unsigned char BranchIndex;
    unsigned char LampIndex;

    unsigned char RF_Rx_PacketFree;       //position to write data to buffer
    unsigned char RF_Rx_PacketData;       //position to read data to buffer
    unsigned char RF_Num_Of_Rx_packet;    //total packet already receiving
    unsigned char RF_Single_packet_Index; //mark position for next byte in single msg
    unsigned char RF_Single_packet_Len;   //count num of received bytes in single msg
} RF_RX_STRUCT;

// ---- API for command lists ----
int xbee_frame_write( void *header, uint8_t headerlen, uint8_t *data, uint8_t datalen);
uint8_t xbee_next_frame_id( void );
void Xbee_App_Router_main(void);
void Xbee_TransmitRequest_exe( uint8_t *databuf, uint8_t datalen, uint8_t *Des_64bit);

uint8_t _xbee_checksum( uint8_t *bytes, uint8_t length, uint8_t initial);
unsigned char RF_Write(unsigned char *buf, unsigned char len);
void xbee_first_cmd_process(uint8_t * frame, uint8_t FrameLen);
void xbee_frame_load( uint8_t rx_char);
void Router_send_status_msg_exe( void );
void Router_send_status_queue(unsigned char long_delay);
void Router_RX_msg_process_Char(unsigned char data);

#endif

