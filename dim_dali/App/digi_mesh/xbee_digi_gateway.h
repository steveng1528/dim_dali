#ifndef XBEE_DIGI_GATEWAY_H
#define XBEE_DIGI_GATEWAY_H


#define Active_COM      COM2

typedef enum xbee_app_global_state{
    XBEE_G_GET_MODULE_CONFIG,
    XBEE_G_GW_DISCOVERY,
    XBEE_G_GW_CONNECTED
}xbee_app_global_state;

enum xbee_dev_rx_state {
    XBEE_RX_STATE_WAITSTART = 0,	///< waiting for initial 0x7E
    XBEE_RX_STATE_LENGTH_MSB,		///< waiting for MSB of length (first byte)
    XBEE_RX_STATE_LENGTH_LSB,		///< waiting for LSB of length (second byte)
    XBEE_RX_STATE_RXFRAME,			///< receiving frame and/or trailing checksum
    XBEE_RX_STATE_CHECKSUM
};
/// Buffer and state variables used for receiving a frame.
typedef struct {
    /// current state of receiving a frame
    enum xbee_dev_rx_state	state;

    /// bytes in frame being read; does not include checksum byte
    u16						bytes_in_frame;

    /// bytes read so far
    u16						bytes_read;

    /// bytes received, starting with frame_type, +1 is for checksum
    u8	frame_data[XBEE_MAX_FRAME_LEN + 1];
} _XBEE_RX;

typedef struct{
    unsigned int  time_delay_In_ms;
    xbee_app_global_state state;
    u8 node_ID[2];
    u8 frame_ID;
}_XBEE_DEV;

#define     MAX_RX_STORED_PACKET    10
#define     MAX_READ_BUFF           20
typedef struct{
    xbee_frame_receive_explicit_t Xbee_Rx_msg[MAX_RX_STORED_PACKET];
    unsigned char lenght[MAX_RX_STORED_PACKET];
    QUEUE_MANAGE q;
    unsigned char ReadSinglePacketIndex;//mark position for next byte in single msg
    unsigned char ReadSinglePacketLen;  //count num of received bytes in single msg
} RF_RX_STRUCT;

int xbee_frame_write( void *header, u8 headerlen, u8 *data, u8 datalen);
u8 xbee_next_frame_id( void );
u8 _xbee_checksum( u8 *bytes, u8 length, u8 initial);
unsigned char RF_Write(unsigned char *buf, unsigned char len);
void xbee_first_cmd_process(u8 * frame, u8 FrameLen);
void xbee_frame_load( u8 rx_char);
void Router_send_status_msg_exe( void );
void GatewayRF_Proccess_Rx_Packet( void );
void Router_RX_msg_process_Char(unsigned char data);
void Print_Inform_To_Host ( xbee_frame_receive_explicit_t  Xbee_Rx_msg);
void Xbee_TransmitRequest_exe( u8 *databuf, u8 datalen, u8 *Des_64bit);
#endif // XBEE_DIGI_GATEWAY_H
