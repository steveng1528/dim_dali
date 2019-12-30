/**
  ******************************************************************************
  * @file    Xbee_Router_ATcmd.c
  * @author  Thai Pham
  * @version V1.0
  * @date    October 01, 2015
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

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "platform_config.h"
#include "platform_typedef.h"
#include "Xbee_Router_ATcmd.h"
#include "Xbee_Digi_Router.h"
#include "timer.h"
#include "Lamp_status.h"

extern void Print_General_To_Host ( unsigned char * ptr, unsigned char length );

/* Private typedef ------------------------------------------------------------*/
/// Format of XBee API frame type 0x10 (#XBEE_FRAME_TRANSMIT); sent
/// from host to XBee.  Note that the network stack does not include a
/// function for sending this frame type -- use an explicit transmit frame
/// instead (type 0x11) with WPAN_ENDPOINT_DIGI_DATA as the source and
/// destination endpoint, DIGI_CLUST_SERIAL as the cluster ID and
/// WPAN_PROFILE_DIGI as the profile ID.  Or, use the xbee_transparent_serial()
/// function from xbee_transparent_serial.c to fill in those fields and
/// send the data.

typedef __packed struct {
	uint8_t			frame_type;			///< XBEE_FRAME_TRANSMIT (0x10)
	uint8_t			frame_id;
	addr64			ieee_address;
	uint16_t			network_address_be;
	uint8_t			broadcast_radius;		///< set to 0 for maximum hop value
	uint8_t			options;			///< combination of XBEE_TX_OPT_* macros
	//uint8_t                RFdata[49];
} xbee_Frame_transmit_t;

/**
  * @brief  General initialize Radio module
  * @param  None
  * @retval None
  */
xbee_Frame_transmit_t Loc_Tx_header = {XBEE_FRAME_TRANSMIT, 0, 0, 0, 0 , 0};
void Xbee_TransmitRequest_exe( u8 *databuf, u8 datalen, u8 *Des_64bit)
{

#ifdef GATEWAY_PRINT_DEBUG
    unsigned char Loc_Host_send_addr[50];
    unsigned char *Loc_Host_ptr = &Loc_Host_send_addr[0];
    unsigned char i,Host_len;
#endif

    // check valid parameter
    if ((! databuf) && (! Des_64bit) && (datalen))
    {
        return ;
    }

    memcpy(Loc_Tx_header.ieee_address.b, Des_64bit, 8);
    Loc_Tx_header.frame_id = xbee_next_frame_id();

    xbee_frame_write(&Loc_Tx_header, sizeof(xbee_Frame_transmit_t), databuf, datalen);

#ifdef GATEWAY_PRINT_DEBUG
    Host_len = (datalen > 40)? 40: datalen;
    memcpy(Loc_Host_ptr, databuf, Host_len);
    Loc_Host_ptr += Host_len;
    *Loc_Host_ptr++ = '-';
    for(i = 0; i < 4; i++)
    {
        SM_Hex2Ascii(Des_64bit[4+i], Loc_Host_ptr);
        Loc_Host_ptr += 2;
    }
    *Loc_Host_ptr = 0x0d;
    Host_len += 10;
    Print_General_To_Host(Loc_Host_send_addr, Host_len);
    
#endif
}


