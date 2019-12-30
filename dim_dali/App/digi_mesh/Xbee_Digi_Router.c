
/**
  ******************************************************************************
  * @file    Xbee_Digi_Router.c
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
#include "stddef.h"
#include "string.h"
#include "platform_config.h"
#include "platform_typedef.h"
#include "Xbee_Router_ATcmd.h"
#include "Xbee_Digi_Router.h"
#include "stm32_Uart_HAL.h"
#include "timer.h"
#include "Lamp_status.h"

/* Global variable ------------------------------------------------------------*/
_XBEE_DEV   Xbee_dev;
_XBEE_RX    Xbee_rx;
RF_RX_STRUCT        RouterRF_Rx_Val;

#define     NODE_ADDRESS_LEN    8
u8 GW_Address[NODE_ADDRESS_LEN] = {0x00, 0x13, 0xA2, 0x00, 0x41, 0x4E, 0x00, 0xC4};

extern LAMP_STATUS_TYPE    Lamp_status_val;
extern LAMP_COMMAND_TYPE   Lamp_cmd_signal;

/* static functions ------------------------------------------------------------*/
// Since we're not using a dynamic frame dispatch table, we need to define
// it here.
xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
    { XBEE_FRAME_LOCAL_AT_RESPONSE, 0, xbee_ATcmd_handle_response },
    { XBEE_FRAME_MODEM_STATUS, 0, xbee_cmd_modem_status },
    { XBEE_FRAME_REMOTE_AT_RESPONSE, 0, xbee_ATcmd_handle_response },
    { XBEE_FRAME_RECEIVE_EXPLICIT, 0, xbee_handle_receive_explicit },
    { XBEE_FRAME_TRANSMIT_STATUS, 0, xbee_handle_transmit_status },
    XBEE_FRAME_TABLE_END
};


/**
  * @brief  General initialize Radio module
  * @param  None
  * @retval None
  */
void Radio_Init(void)
{
    USART_InitTypeDef USART_InitStruct;
    u8 Loc_header_ATcmd[2];
    //u8 Loc_data_ATcmd[2];

    /* USART1 or USART3 configured as follow:
    - BaudRate = 9600 baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
    */
    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    /* disable the uart rx/tx before setting it up */
    Disable_UART(Active_COM);
    
    STM_COMInit(Active_COM, &USART_InitStruct);
    /* Configure CTS pin  */
    STM_CTS_Init();
    
    COM_NVIC_Configuration(Active_COM);     //INIT INTERRUPT FOR UART

    // Reset Radio module
    STM_Spec_GPIO_LOW(RESET_PIN);
    delay_ms(100);
    STM_Spec_GPIO_HIGH(RESET_PIN);
    // give the XBee 500ms to wake up after resetting it (or exit if it
    // receives a packet)
    delay_ms(500);
    
    /* Enable USART */
    Enable_UART(Active_COM);

    RF_init_LED_Blink_cont();

    /*init variable*/
    Xbee_rx.bytes_in_frame = 0;
    Xbee_rx.bytes_read  = 0;
    Xbee_rx.state = XBEE_RX_STATE_WAITSTART;
    
    Enable_Uart_Irq(Active_COM);
    COM_Send_byte( Active_COM, 0x00 ); /*send Null bytes because UART lost 1st byte in Tx*/

    /*get config inside Xbee module*/
    Xbee_dev.time_delay_In_ms = (u16)3000;
    Xbee_dev.frame_ID = 1;
    Loc_header_ATcmd[0] = 0x08;
    Loc_header_ATcmd[1]= xbee_next_frame_id();
    xbee_frame_write(Loc_header_ATcmd, 2, "NI", 2);
    Xbee_dev.state = XBEE_G_GET_MODULE_CONFIG;

    /*Init specific variable for Router*/
    RouterRF_Rx_Val.RF_Rx_Status = START_MSG;
    STM_Spec_GPIO_HIGH(LED1_PIN);

    GW_Address[0] = 0x00;
    GW_Address[1] = 0x13;
    GW_Address[2] = 0xA2;
    GW_Address[3] = 0x00;
    GW_Address[4] = 0x41;
    GW_Address[5] = 0x4E;
    GW_Address[6] = 0x00;
    GW_Address[7] = 0xC4;
}

/**
  * @brief  General initialize Radio module
  * @param  None
  * @retval None
  */
void Xbee_AppRouterSetGWAddr(u8 * address)
{
    u8 tempcount = 0;
    for(int i=0; i < 4; i++){
        if(address[i] == 0xff)
            tempcount++;
    }
    if(tempcount == 4){
        return;
    }else{
        memcpy(GW_Address, address, NODE_ADDRESS_LEN);
    }
}

/**
  * @brief  General initialize Radio module
  * @param  None
  * @retval None
  */
void Xbee_App_Router_main(void)
{
    switch(Xbee_dev.state){
    case XBEE_G_GET_MODULE_CONFIG:

        break;

    case XBEE_G_GW_DISCOVERY:
//        if(TRUE == Get_Timer_expired(XBEE_DISCOVERY_GW)){
//            Router_send_status_msg_exe();
//            //Timer_start(XBEE_DISCOVERY_GW);
//        }
//        break;

    case XBEE_G_GW_CONNECTED:
        if(1 == Get_Timer_expired(DELAY_BEFORE_SEND_TIMER))
        {
            Timer_Stop(DELAY_BEFORE_SEND_TIMER);
            Router_send_status_msg_exe();
        }
        break;
    default: break;
    }
}

/*** BeginHeader xbee_next_frame_id */
/*** EndHeader */
/**
    @brief
    Increment and return current frame ID for a given XBee device.

    Frame IDs go from 1 to 255 and then back to 1.

    @param[in] xbee	XBee device.

    @retval	1-255 Current frame ID (after incrementing) for device
    @retval	0 \a xbee is not a valid XBee device pointer.
*/
u8 xbee_next_frame_id( void )
{

    // frame_id ranges from 1 to 255; if incremented to 0, wrap to 1
    Xbee_dev.frame_ID++;
    if( 0 == Xbee_dev.frame_ID)
    {
        Xbee_dev.frame_ID = 1;
    }

    return Xbee_dev.frame_ID;
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
#define UART_ISR USART3_IRQHandler  //Note: must be modify if changing UART peripheral
void UART_ISR( void )
{
    unsigned short char_rxd;
    if(COM_Get_Error_Status(Active_COM) == SET)
    {
        /*there are some problem, discard this byte*/
        char_rxd = UART_Read_Byte(Active_COM);
    }
    else
    {
        while ( COM_Get_RXIT_Status(Active_COM) == SET)
            /* RX Interrupt Detected */
        {
            char_rxd = UART_Read_Byte(Active_COM);
            char_rxd &= 0xff;
            
            xbee_frame_load(char_rxd);
        }
    }
}

/*** BeginHeader _xbee_checksum */
/*** EndHeader */
/**
    @internal
    @brief
    Calculate the checksum for an XBee frame.

                    This function actually
                    subtracts bytes from 0xFF, in order to determine the proper
                    checksum byte so all bytes added together result in 0x00.

                    Frame checksums start with the frame type (first byte after
                    frame length).

                    If calculating a checksum for an outgoing packet, sum all of the
                    bytes using this function, and send the low byte of the result.

                    If verifying a received packet, summing all of the bytes after
                    the length, including the packet's checksum byte, should result
                    in 0x00.

                    Should only be used internally by this library.

    @param[in]	bytes		Buffer of bytes to add to sum, assumed non-NULL.

    @param[in]	length	Number of bytes to add.

    @param[in]	initial	Starting checksum value.  Use 0xFF to start or the
                                result  of the previous call if summing multiple
                                blocks of data.

    @return	Value from 0x00 to 0xFF that, when added to all bytes previously
                    checksummed, results in a low byte of 0x00.

*/

// Function name in parenthesis so platforms can provide an inline assembly
// version as a replacement, and include a function macro to override this
// function in their platform.h.  See Rabbit
u8 _xbee_checksum( u8 *bytes, u8 length, u8 initial)
{
    uint8_t i;
    uint8_t checksum;
    u8 *p;

    checksum = initial;
    p = (u8 *)bytes;
    for ( i = length; i; ++p, --i)
    {
        checksum -= *p;
    }

    return checksum;
}

/*** BeginHeader xbee_frame_write */
/*** EndHeader */
/**
    @brief
    Copies a frame into the transmit serial buffer to send to an
    XBee module.

    By default, xbee_frame_write() checks the /CTS signal from the XBee before
    attempting to send, and will return -EBUSY if the XBee has deasserted CTS.
    Use xbee_dev_flowcontrol() to disable this check (necessary on a system
    without a connection to the XBee module's /CTS signal).

    @param[in]	headerlen	Number of header bytes to send (starting with
                                    address passed in \a header).  Ignored if
                                    \a header is \c NULL.

    @param[in]	data			Address of frame payload or \c NULL if the entire
                                    frame content is stored in the header bytes
                                    (\a header and \a headerlen).

    @param[in]	datalen		Number of payload bytes to send (starting with
                                    address passed in \a data).  Ignored if \a data
                                    is \c NULL.

    @retval	0				Successfully queued frame in transmit serial buffer.
    @retval	-EINVAL		\a xbee is \c NULL or invalid flags passed
    @retval	-ENODATA		No data to send (\a headerlen + \a datalen == 0).
    @retval	-EBUSY		Transmit serial buffer is full, or XBee is not
                                accepting serial data (deasserting /CTS signal).
    @retval	-EMSGSIZE	Serial buffer can't ever send a frame this large.

    @sa xbee_dev_init(), xbee_serial_write(), xbee_dev_flowcontrol()
*/

int xbee_frame_write( void *header, u8 headerlen, u8 *data, u8 datalen)
{
    u8 Loc_Tx_Buf[3];

    u8 framesize;
    u8 checksum = 0xFF;

    // check valid parameter
    if ((! data) && (! header) && (datalen) && (headerlen))
    {
        //datalen = 0;		// if data is NULL, set datalen to 0
        return EINVAL;
    }

    // Make sure XBee is asserting CTS and verify that the transmit serial buffer
    // has enough room for the frame (payload + 3-byte header + 1-byte checksum).

    framesize = headerlen + datalen + 3 + 1;

    // Send 0x7E (start frame marker) and 16-bit length
    Loc_Tx_Buf[0] = 0x7E;
    Loc_Tx_Buf[1] = 0;
    Loc_Tx_Buf[2] = (headerlen + datalen);

    checksum = _xbee_checksum( (u8 *)header, headerlen, checksum);
    checksum = _xbee_checksum( data, datalen, checksum);

    RF_Write(&Loc_Tx_Buf[0], 3);

    RF_Write((u8 *)header, headerlen);

    RF_Write(data, datalen);
    
    RF_Write(&checksum, 1);
    
    return framesize;
}

/**
  * @brief  write a buffer to Radio module
  *         this function just forward via HW without any modification .
  * @param  *buf
  *          len: must be less than 255 (unsigned char)
  * @retval None
  */
unsigned char RF_Write(unsigned char *buf, unsigned char len)
{
    unsigned char i;
    
    for(i=0; i<len; i++)
    {
        //if(GPIO_ReadInputDataBit( CTS_GPIO_PORT, CTS_GPIO_PIN ) == Bit_RESET)
        {
            COM_Send_byte( Active_COM, buf[i] );
        }
        /*else
        {
            while(GPIO_ReadInputDataBit( CTS_GPIO_PORT, CTS_GPIO_PIN ) != Bit_RESET);
        }*/
    }
    return len;
}

/**
  * @brief  call from ISR with Rx character. This funtion process every single character
  *
  * @param  None
  *
  * @retval None
  */
void xbee_frame_load( u8 rx_char)
{
    // Use xbee_serial API to load multiple bytes at a time.

    // Based on state, do one of the following:

    // 1) Waiting for start of frame:
    // Scan through serial buffer until 0x7e byte is found.
    // Advance to next state.

    // 2) Waiting for length:
    // Wait until 2 bytes in serial buffer, read into xbee->rx.bytes_in_frame.

    // 3) Waiting for (<length> + 1) bytes of data:
    // Read as many bytes as possible from the serial buffer and into the
    // xbee->rx.frame_data[].  Once all bytes have been read, calculate and
    // verify checksum and then hand off to dispatcher.
    switch(Xbee_rx.state)
    {
    case XBEE_RX_STATE_WAITSTART:
        if(0x7E == rx_char){
            //find start of frame, go to next state
            Xbee_rx.state = XBEE_RX_STATE_LENGTH_MSB;
        }
        break;
    case XBEE_RX_STATE_LENGTH_MSB:
        if(0x7E != rx_char){
            Xbee_rx.bytes_in_frame = (u16)rx_char<<8;
            Xbee_rx.state = XBEE_RX_STATE_LENGTH_LSB;
        }else{
            // MSB of length can never be 0x7E, consider it to be the new
            // start-of-frame character and recheck for the length.

            /* do nothing*/
        }
        break;
    case XBEE_RX_STATE_LENGTH_LSB:
        Xbee_rx.bytes_in_frame += rx_char;
        if(Xbee_rx.bytes_in_frame > XBEE_MAX_FRAME_LEN || Xbee_rx.bytes_in_frame < 2)
        {
            // this isn't a valid frame, go back to looking for start marker
            if(0x7E == rx_char){
                // Handle case of 0x7E 0xXX 0x7E where second 0x7E is actual
                // start of frame.
                Xbee_rx.state = XBEE_RX_STATE_LENGTH_MSB;
            }else{
                Xbee_rx.state = XBEE_RX_STATE_WAITSTART;
            }
        }else{
            //still ok, go ahead
            Xbee_rx.state = XBEE_RX_STATE_RXFRAME;
            Xbee_rx.bytes_read = 0;
        }
        break;
    case XBEE_RX_STATE_RXFRAME:
        Xbee_rx.frame_data[Xbee_rx.bytes_read++] = rx_char;
        if(Xbee_rx.bytes_read == Xbee_rx.bytes_in_frame)
            Xbee_rx.state = XBEE_RX_STATE_CHECKSUM;
        break;
    case XBEE_RX_STATE_CHECKSUM:
        Xbee_rx.state = XBEE_RX_STATE_WAITSTART;
        Xbee_rx.frame_data[Xbee_rx.bytes_read++] = rx_char;
        if(_xbee_checksum(Xbee_rx.frame_data, Xbee_rx.bytes_read, 0xff))
        {
            // checksum failed, discard this frame
        }else{
            xbee_first_cmd_process(Xbee_rx.frame_data, Xbee_rx.bytes_in_frame);
        }
        break;
    default: break;
    }
}

/**
  * @brief  Function called by _xbee_frame_load() to dispatch any frames read.
  *
  * @param  None
  *
  * @retval None
  */
void xbee_first_cmd_process(u8 * frame, u8 FrameLen)
{
    u8 frametype, frameid;
    xbee_dispatch_table_entry_t *entry;
    u8 dispatched;

    if (! ( frame && FrameLen))
    {
        // Since this is an internal API, always called correctly, this should
        // not happen and could be changed to an assert.  Currently part of
        // unit tests for this module.
        return ;
    }
    // first byte of <frame> is the frametype, second is frame ID
    frametype = ((u8 *)frame)[0];
    frameid = ((u8 *)frame)[1];

    dispatched = 0;
    for (entry = xbee_frame_handlers; entry->frame_type != 0xFF; ++entry)
    {
        if (entry->frame_type == frametype)
        {
            if (! entry->frame_id || entry->frame_id == frameid)
            {
                ++dispatched;
                entry->handler( frame, FrameLen);
                break; /*just accept the first detection*/
            }
        }
    }
}


/**	@internal
    @brief
    Callback handler registered for frame types
    #XBEE_FRAME_LOCAL_AT_RESPONSE (0x88) and
    #XBEE_FRAME_REMOTE_AT_RESPONSE (0x97).

    Should only be called by the frame dispatcher and unit tests.

    See the function help for xbee_frame_handler_fn() for full
    documentation on this function's API.
*/
int xbee_ATcmd_handle_response( u8 *rawframe, u8  length )
{
    xbee_frame_at_response_t *frame = (xbee_frame_at_response_t *)rawframe;
    u8 value_length;
    if (frame == NULL)
    {
        // extra checking, since stack always passes a valid frame
        return EINVAL;
    }
    switch(frame->frame_type)
    {
    case XBEE_FRAME_LOCAL_AT_RESPONSE:
        if(Xbee_dev.state == XBEE_G_GET_MODULE_CONFIG){
            STM_Spec_GPIO_HIGH(LED2_PIN);
            value_length = (u8)(length - offsetof(xbee_frame_local_at_resp_t, value));
            if(4 == value_length){
                Xbee_dev.node_ID[0] =  (frame->local.value[0]-0x30);
                Xbee_dev.node_ID[1] =  (frame->local.value[2]-0x30)*10 + (frame->local.value[3]-0x30);
                Lamp_status_val.name[0] = Xbee_dev.node_ID[0];
                Lamp_status_val.name[1] = Xbee_dev.node_ID[1];
                Xbee_dev.time_delay_In_ms = ((unsigned int)(Xbee_dev.node_ID[1])*1000);
            }
            Xbee_dev.state = XBEE_G_GW_DISCOVERY;
//            Timer_create(XBEE_DISCOVERY_GW, Xbee_dev.time_delay_In_ms);
//            Timer_start( XBEE_DISCOVERY_GW );
            /*create timer for delay before response a status msg*/
            Timer_create(DELAY_BEFORE_SEND_TIMER, Xbee_dev.time_delay_In_ms);
        }else{
            /*discard it*/
        }
        break;
    case XBEE_FRAME_REMOTE_AT_RESPONSE:

        break;
    default: break;
    }
    return EINVAL;
}

/*** BeginHeader xbee_cmd_modem_status */
/*** EndHeader */
/**
    @internal
    @brief
    Receive modem status frames and update our network address (and payload
    size) on state changes.

    @see xbee_frame_handler_fn()
*/
int xbee_cmd_modem_status( u8 *rawframe, u8  length )
{

    return EINVAL;
}

/**
    Process XBee "Receive Explicit" frames (type 0x91) and hand
    off to wpan_envelope_dispatch() for further processing.

    Please view the function help for xbee_frame_handler_fn() for details
    on this function's parameters and possible return values.
*/
int xbee_handle_receive_explicit( u8 *rawframe, u8  length )
{
    xbee_frame_receive_explicit_t *frame = (xbee_frame_receive_explicit_t *)rawframe;
    u8 payload_len,i;

    if (frame == NULL)
    {
        return EINVAL;
    }
    if (length < offsetof( xbee_frame_receive_explicit_t, payload))
    {
        // invalid frame -- should always be at least as long as payload field
        return EBADMSG;
    }
    payload_len = length - offsetof( xbee_frame_receive_explicit_t, payload);
    for(i = 0; i < payload_len; i++){
        Router_RX_msg_process_Char(frame->payload[i]);
    }
    //copy 8 bytes addres to GW address for Router reply msg
    memcpy(&GW_Address[0], &frame->ieee_address.b[0], 8);

    RF_RX_LED_Blinking();
    return length;
}

/*** BeginHeader _xbee_handle_transmit_status */
/*** EndHeader */
// see xbee/device.h for documentation
xbee_frame_transmit_status_t Xbee_dev_Tx_status;
int xbee_handle_transmit_status( u8 *rawframe, u8  length )
{
    xbee_frame_transmit_status_t *frame = (xbee_frame_transmit_status_t *)rawframe;

    Xbee_dev_Tx_status.retries = frame->retries;
    Xbee_dev_Tx_status.delivery = frame->delivery;
    Xbee_dev_Tx_status.discovery = frame->discovery;
    
    if(XBEE_TX_DELIVERY_SUCCESS == Xbee_dev_Tx_status.delivery){
        if(Xbee_dev.state == XBEE_G_GW_DISCOVERY){
            RF_init_LED_Stop_Blink();
            Xbee_dev.state = XBEE_G_GW_CONNECTED;
            Timer_Stop(XBEE_DISCOVERY_GW);
        }
    }else{
//        Timer_start(XBEE_DISCOVERY_GW);
    }

    return EINVAL;
}

/**
  * @brief  call from ISR
  *
  * @param  None
  *
  * @retval None
  */
void Router_send_status_msg_exe( void )
{
    unsigned char Loc_TxPacket[16];
    Loc_TxPacket[0] = Router_STATUS_REPLY;
    Loc_TxPacket[1] = Xbee_dev.node_ID[0];
    Loc_TxPacket[2] = Xbee_dev.node_ID[1];
    Loc_TxPacket[3] = Lamp_status_val.Voltage[0];
    Loc_TxPacket[4] = Lamp_status_val.Voltage[1];
    Loc_TxPacket[5] = Lamp_status_val.Current[0];
    Loc_TxPacket[6] = Lamp_status_val.Current[1];
    Loc_TxPacket[7] = Lamp_status_val.Power[0];
    Loc_TxPacket[8] = Lamp_status_val.Power[1];
    Loc_TxPacket[9] = Lamp_status_val.Temperature;
    Loc_TxPacket[10] = Lamp_status_val.Dim_Level;
    Loc_TxPacket[11] = Lamp_status_val.lamp_status;
    memset(&Loc_TxPacket[12], 0 , 4);
    Xbee_TransmitRequest_exe(Loc_TxPacket, 16, GW_Address);
    
    RF_TX_LED_Blinking();
    
}

/**
  * @brief  call from ISR
  *
  * @param  None
  *
  * @retval None
  */
void Router_send_status_queue(unsigned char long_delay)
{
    if(1 == long_delay){
        Timer_create( DELAY_BEFORE_SEND_TIMER, Xbee_dev.time_delay_In_ms);
    }else{
        Timer_create( DELAY_BEFORE_SEND_TIMER, 100);
    }
    Timer_start(DELAY_BEFORE_SEND_TIMER);
}


/**
  * @brief  call from ISR
  *
  * @param  None
  *
  * @retval None
  */
void Router_RX_msg_process_Char(unsigned char data)
{
    
    switch (RouterRF_Rx_Val.RF_Rx_Status)
    {
    case START_MSG:
        RouterRF_Rx_Val.cmd_type = data;
        RouterRF_Rx_Val.RF_Rx_Status = BRANCH_INDEX;
        break;
    case BRANCH_INDEX:
        RouterRF_Rx_Val.BranchIndex = data;
        RouterRF_Rx_Val.RF_Rx_Status = LAMP_INDEX;
        break;
    case LAMP_INDEX:
        RouterRF_Rx_Val.LampIndex = data;
        switch(RouterRF_Rx_Val.cmd_type)
        {
        case GW_READ_STATUS:
            if((RouterRF_Rx_Val.BranchIndex == 0xFF)&&(RouterRF_Rx_Val.LampIndex== 0xFF)){
                Router_send_status_queue(1);
            }else{
                Router_send_status_queue(0);
            }
            RouterRF_Rx_Val.RF_Rx_Status = START_MSG;
            break;
        case GW_SET_DIM:
            RouterRF_Rx_Val.RF_Rx_Status = SET_DIM_CMD;
            //RouterRF_Rx_Val.cmd_Length = 1;
            break;
        case GW_SET_CONFIG_RF_MODULE:
            RouterRF_Rx_Val.RF_Rx_Status = SET_CONFIG_RF_MODULE_CMD;
            RouterRF_Rx_Val.cmd_Length = 2;
            break;
        case GW_RESET_RF_MODULE:
            //RF_module_config.Is_reset_receive = 1;
            RouterRF_Rx_Val.RF_Rx_Status = START_MSG;
            Timer_create(ROUTER_DELAY_BEFORE_RESET, 10000);
            Timer_start(ROUTER_DELAY_BEFORE_RESET);
            break;
        case GW_SET_NAME:
            Lamp_status_val.name[0] = RouterRF_Rx_Val.BranchIndex;
            Lamp_status_val.name[1] = RouterRF_Rx_Val.LampIndex;
            //SetNameOfLamp(RouterRF_Rx_Val.BranchIndex, RouterRF_Rx_Val.LampIndex);
            Router_send_status_queue(0);
            break;
        case GW_SET_TIME_REPLY:
            RouterRF_Rx_Val.RF_Rx_Status = SET_TIMEREPLY_MODE;
            break;

        default: break;
        }
        if(RouterRF_Rx_Val.cmd_type == GW_READ_SCHEDULE){

        }else{

        }

        break;
    case SET_DIM_CMD:
        RouterRF_Rx_Val.RF_Rx_Status = START_MSG;
        Lamp_cmd_signal.Is_signal = 1;
        Lamp_cmd_signal.DIM_level = data;
        lamp_set_DimLevel(&Lamp_cmd_signal.DIM_level);
        if((RouterRF_Rx_Val.BranchIndex == 0xFF)&&\
                (RouterRF_Rx_Val.LampIndex == 0xFF))
        {
            Router_send_status_queue(1);
        }
        else if((RouterRF_Rx_Val.BranchIndex == Lamp_status_val.name[0])&&\
                (RouterRF_Rx_Val.LampIndex == Lamp_status_val.name[1]))
        {
            Router_send_status_queue(0);
        }
        break;
    case SET_TIMEREPLY_MODE:
#ifdef NO_DEFINE
        RouterRF_Rx_Val.RF_Rx_Status = START_MSG;
        if(1 == data){
            Xbee_dev.time_delay_In_ms = (3000 + ((TimePeriodInReply)*(Lamp_status_val.name[1]/2)));
        }else if(2 == data){
            Xbee_dev.time_delay_In_ms = (3000 + ((TimePeriodInReply/2)*(Lamp_status_val.name[1]/2)));
        }else if(3 == data){
            Xbee_dev.time_delay_In_ms = ((TimePeriodInReply/2)*(Lamp_status_val.name[1]));
        }else{
            Xbee_dev.time_delay_In_ms = (TimePeriodInReply*(Lamp_status_val.name[1]));
        }
        Timer_create( DELAY_BEFORE_SEND_TIMER, Xbee_dev.time_delay_In_ms);
        if((RouterRF_Rx_Val.BranchIndex == 0xFF)&&\
                (RouterRF_Rx_Val.LampIndex == 0xFF))
        {
            Router_send_status_queue(1);
        }
        else if((RouterRF_Rx_Val.BranchIndex == Lamp_status_val.name[0])&&\
                (RouterRF_Rx_Val.LampIndex == Lamp_status_val.name[1]))
        {
            Router_send_status_queue(0);
        }
#endif
        break;

    default: break;
    }
}

