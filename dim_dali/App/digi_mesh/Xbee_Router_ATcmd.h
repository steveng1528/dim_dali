/**
  ******************************************************************************
  * @file    Xbee_Router_ATcmd.h
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

#ifndef  XBEE_ATCMD_ROUTER_H
#define  XBEE_ATCMD_ROUTER_H

#include "stm32f1xx_hal.h"
#include "platform_typedef.h"

/** Possible values for the \c frame_type field of frames sent to and
	from the XBee module.  Values with the upper bit set (0x80) are frames
	we receive from the XBee module.  Values with the upper bit clear are
	for frames we send to the XBee.
*/
enum xbee_frame_type {
	/// Send an AT Command to the local device (see xbee_atcmd.c,
	/// xbee_header_at_request_t). [ZigBee, DigiMesh]
	XBEE_FRAME_LOCAL_AT_CMD					= 0x08,

	/// Queue an AT command for batch processing on the local device.
	/// [ZigBee, DigiMesh]
	XBEE_FRAME_LOCAL_AT_CMD_Q				= 0x09,

	/// Send data to a default endpoint and cluster on a remote device.
	/// [ZigBee, DigiMesh, not Smart Energy]
	XBEE_FRAME_TRANSMIT						= 0x10,

	/// Send data to a specific endpoint and cluster on a remote device
	/// (see xbee_wpan.c). [ZigBee, DigiMesh]
	XBEE_FRAME_TRANSMIT_EXPLICIT			= 0x11,

	/// Send an AT command to a remote device on the network (see xbee_atcmd.c,
	/// xbee_header_at_request_t). [ZigBee, DigiMesh, not Smart Energy]
	XBEE_FRAME_REMOTE_AT_CMD				= 0x17,

	/// Create Source Route (used with many-to-one routing) [ZigBee]
	XBEE_FRAME_CREATE_SRC_ROUTE			= 0x21,

	/// Register Joining Device (add device to trust center's key table)
	/// [Smart Energy, coordinator]
	XBEE_FRAME_REG_JOINING_DEV				= 0x24,

	/// Response from local device to AT Command (see xbee_atcmd.c,
	/// xbee_cmd_response_t). [ZigBee, DigiMesh]
	XBEE_FRAME_LOCAL_AT_RESPONSE			= 0x88,

	/// Current modem status (see xbee_frame_modem_status_t). [DigiMesh, ZigBee]
	XBEE_FRAME_MODEM_STATUS					= 0x8A,

	/// Frame sent upon completion of a Transmit Request. [DigiMesh, ZigBee]
	XBEE_FRAME_TRANSMIT_STATUS				= 0x8B,

	/// Route Information Frame, sent for DigiMesh unicast transmissions with
	/// NACK or Trace Route Enable transmit options set. [DigiMesh]
	XBEE_FRAME_ROUTE_INFORMATION			= 0x8D,

	/// Output when a node receives an address update frame and modifies its
	/// DH/DL registers. [DigiMesh]
	XBEE_FRAME_AGGREGATE_ADDRESSING		= 0x8E,

	/// Data received on the transparent serial cluster, when ATAO is set to 0.
	/// [ZigBee, DigiMesh]
	XBEE_FRAME_RECEIVE						= 0x90,		// ATAO == 0

	/// Data received for specific endpoint/cluster (see xbee_wpan.c), when
	/// ATAO is non-zero. [ZigBee, DigiMesh]
	XBEE_FRAME_RECEIVE_EXPLICIT			= 0x91,		// ATAO != 0

	/// [ZigBee, not Smart Energy]
	XBEE_FRAME_IO_RESPONSE					= 0x92,

	/// [ZigBee, not Smart Energy]
	XBEE_FRAME_SENDOR_READ					= 0x94,

	/// [ZigBee, DigiMesh, not Smart Energy]
	XBEE_FRAME_NODE_ID						= 0x95,

	/// Response from remote device to AT Command (see xbee_atcmd.c,
	/// xbee_cmd_response_t). [ZigBee, DigiMesh, not Smart Energy]
	XBEE_FRAME_REMOTE_AT_RESPONSE			= 0x97,

	/// Over-the-Air Firmware Update Status [ZigBee, not Smart Energy]
	XBEE_FRAME_FW_UPDATE_STATUS			= 0xA0,

	/// Route records received in response to a Route Request. [ZigBee]
	XBEE_FRAME_ROUTE_RECORD					= 0xA1,

	/// Information on device authenticated on Smart Energy network.
	/// [Smart Energy, coordinator]
	XBEE_FRAME_DEVICE_AUTHENTICATED		= 0xA2,

	/// Many-to-One Route Request Indicator [ZigBee]
	XBEE_FRAME_ROUTE_REQUEST_INDICATOR	= 0xA3,

	/// Frame sent in response to Register Joining Device frame
	/// (XBEE_FRAME_REG_JOINING_DEV). [Smart Energy, coordinator]
	XBEE_FRAME_REG_JOINING_DEV_STATUS	= 0xA4,

	/// Frame notifying trust center that a device has attempted to
	/// join, rejoin or leave the network.  Enabled by setting bit 1 of ATDO.
	/// [Smart Energy, coordinator]
	XBEE_FRAME_JOIN_NOTIFICATION_STATUS	= 0xA5,
};

/**
	@name
	Values for the \c options field of many receive frame types.
*/
//@{
/// XBee Receive Options: packet was acknowledged [ZigBee, DigiMesh]
#define XBEE_RX_OPT_ACKNOWLEDGED    0x01

/// XBee Receive Options: broadcast packet [ZigBee, DigiMesh]
#define XBEE_RX_OPT_BROADCAST       0x02

/// XBee Receive Options: APS-encrypted packet [ZigBee]
#define XBEE_RX_OPT_APS_ENCRYPT		0x20

/// XBee Receive Options: packet from end device (if known) [ZigBee]
#define XBEE_RX_OPT_FROM_END_DEVICE	0x40		// appeared in ZB 2x7x

/// XBee Receive Options: Mask for transmission mode [DigiMesh]
#define XBEE_RX_OPT_MODE_MASK						0xC0

	/// XBee Receive Options: Mode not specified [DigiMesh]
	#define XBEE_RX_OPT_MODE_NONE						(0)

	/// XBee Receive Options: Point-Multipoint [DigiMesh]
	#define XBEE_RX_OPT_MODE_POINT_MULTIPOINT		(1<<6)

	/// XBee Receive Options: Repeater Mode [DigiMesh]
	#define XBEE_RX_OPT_MODE_REPEATER				(2<<6)

	/// XBee Receive Options: DigiMesh (not available on 10k product) [DigiMesh]
	#define XBEE_RX_OPT_MODE_DIGIMESH				(3<<6)
//@}

/// Max payload for all supported XBee types is 256 bytes.  Actual firmware used
/// may affect that size, and even enabling encryption can have an affect.
/// Smart Energy and ZigBee are limited to 128 bytes, DigiMesh is 256 bytes.
#ifndef XBEE_MAX_RFPAYLOAD
	#define XBEE_MAX_RFPAYLOAD 128
#endif

/// Max Frame Size, including type, is for 0x91, Receive Explicit.  Note that
/// this is only for received frames -- we send 0x11 frames with 20 byte header.
#define XBEE_MAX_FRAME_LEN		(XBEE_MAX_RFPAYLOAD + 18)

enum xbee_dev_flags
{
	XBEE_DEV_FLAG_CMD_INIT			= 0x0001,	///< xbee_cmd_init called
	XBEE_DEV_FLAG_QUERY_BEGIN		= 0x0002,	///< started querying device
	XBEE_DEV_FLAG_QUERY_DONE		= 0x0004,	///< querying completed
	XBEE_DEV_FLAG_QUERY_ERROR		= 0x0008,	///< querying timed out or error
	XBEE_DEV_FLAG_QUERY_REFRESH	= 0x0010,	///< need to re-query device
	XBEE_DEV_FLAG_QUERY_INPROGRESS= 0x0020,	///< query is in progress

	XBEE_DEV_FLAG_IN_TICK			= 0x0080,	///< in xbee_dev_tick

	XBEE_DEV_FLAG_COORDINATOR		= 0x0100,	///< Node Type is Coordinator
	XBEE_DEV_FLAG_ROUTER				= 0x0200,	///< Node Type is Router
	XBEE_DEV_FLAG_ENDDEV				= 0x0400,	///< Node Type is End Device
	XBEE_DEV_FLAG_ZNET				= 0x0800,	///< Firmware is ZNet
	XBEE_DEV_FLAG_ZIGBEE				= 0x1000,	///< Firmware is ZigBee
	XBEE_DEV_FLAG_DIGIMESH			= 0x2000,	///< Firmware is DigiMesh

	// (cast to int required by Codewarrior/HCS08 platform if enum is signed)
	XBEE_DEV_FLAG_USE_FLOWCONTROL	= (int)0x8000,	///< Check CTS before sending
};

enum xbee_dev_mode {
	XBEE_MODE_UNKNOWN = 0,	///< Haven't started communicating with XBee yet
	XBEE_MODE_BOOTLOADER,	/**< XBee is in the bootloader, not running
											firmware */

	// Modes used by "AT firmware" and some bootloaders:
	XBEE_MODE_API,				///< XBee is using API firmware
	XBEE_MODE_IDLE,			///< idle mode, data sent is passed to remote XBee
	XBEE_MODE_PRE_ESCAPE,	///< command mode, can send AT commands to XBee
	XBEE_MODE_POST_ESCAPE,	///< wait for guard-time ms before sending +++
	XBEE_MODE_COMMAND,		///< wait guard-time ms for "OK\r" before command mode
	XBEE_MODE_WAIT_IDLE,		///< waiting for OK response to ATCN command
	XBEE_MODE_WAIT_RESPONSE	///< sent a command and now waiting for a response
};

/** @name
	Values for \c status member of xbee_frame_modem_status_t.
	@{
*/
/// XBee Modem Status: Hardware reset [ZigBee and DigiMesh]
#define XBEE_MODEM_STATUS_HW_RESET					0x00
/// XBee Modem Status: Watchdog timer reset [ZigBee and DigiMesh]
#define XBEE_MODEM_STATUS_WATCHDOG					0x01
/// XBee Modem Status: Joined network (routers and end devices) [ZigBee]
#define XBEE_MODEM_STATUS_JOINED						0x02
/// XBee Modem Status: Disassociated (left network) [ZigBee]
#define XBEE_MODEM_STATUS_DISASSOC					0x03
/// XBee Modem Status: Coordinator started [ZigBee]
#define XBEE_MODEM_STATUS_COORD_START				0x06
/// XBee Modem Status: Network security key was updated [ZigBee]
#define XBEE_MODEM_STATUS_NETWORK_KEY_UPDATED	0x07
/// XBee Modem Status: Network Woke Up [DigiMesh]
#define XBEE_MODEM_STATUS_WOKE_UP					0x0B
/// XBee Modem Status: Network Went To Sleep [DigiMesh]
#define XBEE_MODEM_STATUS_SLEEPING					0x0C
/// XBee Modem Status: Voltage supply limit exceeded (XBee-PRO only) [ZigBee]
#define XBEE_MODEM_STATUS_OVERVOLTAGE				0x0D
/// XBee Modem Status: Key establishment complete [Smart Energy]
#define XBEE_MODEM_STATUS_KEY_ESTABLISHED			0x10
/// XBee Modem Status: Modem config changed while join in progress [ZigBee]
#define XBEE_MODEM_STATUS_CONFIG_CHANGE_IN_JOIN	0x11
/// XBee Modem Status: Network stack error [ZigBee]
#define XBEE_MODEM_STATUS_STACK_ERROR				0x80
//@}

/** @name Function Pointer Prototypes
	Function pointer prototypes, forward declaration using "struct xbee_dev_t"
	instead of "xbee_dev_t" since we use the types in the xbee_dev_t definition.
*/
//@{

/**
	@brief
	Standard API for an XBee frame handler in xbee_frame_handlers global.
	These functions are only called when xbee_dev_tick() or wpan_tick()
	are called and a complete frame is ready for processing.

	@note 		There isn't an actual xbee_frame_handler_fn function in
					the XBee libraries.  This documentation exists as a template
					for writing frame handlers.

	@param[in] xbee
					XBee device that received frame.
	@param[in] frame
					Pointer to frame data.  Data starts with the frame type (the
					0x7E start byte and frame length are stripped by lower layers
					of the driver).
	@param[in] length
					Number of bytes in frame.
	@param[in] context
					Handler-specific "context" value, chosen when the handler was
					registered with xbee_frame_handler_add.

	@retval	0	successfully processed frame
	@retval	!0	error processing frame
*/
/*
					Possible errors that will need unique -Exxx return values:
	            -	Invalid xbee_dev_t
	            -	No wpan_if assigned to xbee_dev_t
	            -	Invalid length (must be > 0)
	            -	Frame pointer is NULL

	@todo			What will _xbee_frame_dispatch do with those return values?
					Does there need to be a return value equivalent to "thanks
					for that frame, but please remove me from the dispatch table"?
					Right now, the dispatcher is ignoring the return value.
					Could be useful when registering to receive status information
					on outbound frames -- once we have status, we don't need
					to receive any additional notifications.
*/
typedef int (*xbee_frame_handler_fn)(
	uint8_t 					*frame,
	uint8_t 					length
);

typedef struct xbee_dispatch_table_entry {
	uint8_t						frame_type;	///< if 0, match all frames
	uint8_t						frame_id;	///< if 0, match all frames of this type
	xbee_frame_handler_fn	handler;
} xbee_dispatch_table_entry_t;

//extern xbee_dispatch_table_entry_t xbee_frame_handlers[];

#define XBEE_FRAME_TABLE_END		{ 0xFF, 0, NULL }

/// Datatype used for passing and storing XBee AT Commands.  Allows printing
/// (e.g., printf( "%.2s", foo.str)), easy copying (bar.w = foo.w) and
/// easy comparison (bar.w == foo.w).
typedef union xbee_at_cmd {
	char		str[2];
	uint16_t		w;
} xbee_at_cmd_t;

/// Format of XBee API frame type 0x90 (#XBEE_FRAME_RECEIVE);
/// received from XBee by host.
typedef PACKED_STRUCT xbee_frame_receive_t {
	uint8_t			frame_type;				///< XBEE_FRAME_RECEIVE (0x90)
	addr64			ieee_address;
	uint16_t			network_address_be;
	uint8_t			options;					///< bitfield, see XBEE_RX_OPT_xxx macros
	uint8_t			payload[1];				///< multi-byte payload
} xbee_frame_receive_t;


/// Format of XBee API frame type 0x91 (#XBEE_FRAME_RECEIVE_EXPLICIT);
/// received from XBee by host.
typedef PACKED_STRUCT xbee_frame_receive_explicit_t {
	uint8_t			frame_type;				///< XBEE_FRAME_RECEIVE_EXPLICIT (0x91)
	addr64			ieee_address;
	uint16_t			network_address_be;
	uint8_t			source_endpoint;
	uint8_t			dest_endpoint;
	uint16_t			cluster_id_be;
	uint16_t			profile_id_be;
	uint8_t			options;					///< bitfield, see XBEE_RX_OPT_xxx macros
	uint8_t			payload[20];				///< multi-byte payload
} xbee_frame_receive_explicit_t;

/// Format of XBee API frame type 0x11 (#XBEE_FRAME_TRANSMIT_EXPLICIT); sent
/// from host to XBee.
typedef PACKED_STRUCT xbee_header_transmit_explicit_t {
	uint8_t			frame_type;				///< XBEE_FRAME_TRANSMIT_EXPLICIT (0x11)
	uint8_t			frame_id;
	addr64			ieee_address;
	uint16_t			network_address_be;
	uint8_t			source_endpoint;
	uint8_t			dest_endpoint;
	uint16_t			cluster_id_be;
	uint16_t			profile_id_be;
	uint8_t			broadcast_radius;		///< set to 0 for maximum hop value
	uint8_t			options;					///< combination of XBEE_TX_OPT_* macros
} xbee_header_transmit_explicit_t;


typedef PACKED_STRUCT xbee_header_local_at_resp {
   /// #XBEE_FRAME_LOCAL_AT_RESPONSE (0x88)
	uint8_t				frame_type;

	/// ID from request, used to match response to request.
	uint8_t				frame_id;
	xbee_at_cmd_t		command;

	/// See enum #xbee_at_resp_status.  Note that DigiMesh uses the upper
	/// nibble for additional flags -- use XBEE_AT_RESP_STATUS() macro when
	/// comparing this field to the xbee_at_resp_status enum.
	uint8_t				status;

} xbee_header_local_at_resp_t;

typedef PACKED_STRUCT xbee_header_remote_at_resp {
   /// #XBEE_FRAME_REMOTE_AT_RESPONSE (0x97)
	uint8_t				frame_type;

	/// ID from request, used to match response to request.
	uint8_t				frame_id;

	/// 64-bit IEEE address (big-endian) of responder.
	addr64				ieee_address;

	/// 16-bit network address (big-endian) of responder.
	uint16_t				network_address_be;

	/// Command from original request.
	xbee_at_cmd_t		command;

	/// See enum #xbee_at_resp_status.  Note that DigiMesh uses the upper
	/// nibble for additional flags -- use XBEE_AT_RESP_STATUS() macro when
	/// comparing this field to the xbee_at_resp_status enum.
	uint8_t				status;
} xbee_header_remote_at_resp_t;


/// Response to an AT Command sent to the local serially-connected XBee.
typedef PACKED_STRUCT xbee_frame_local_at_resp {

   xbee_header_local_at_resp_t header;
	/// First byte of multi-byte value.
	uint8_t				value[20]; //currently max bytes is "NI" cmd with 20bytes length
} xbee_frame_local_at_resp_t;

/// Response to an AT Command sent to a remote XBee.
typedef PACKED_STRUCT xbee_frame_remote_at_resp {
   xbee_header_remote_at_resp_t header;

	/// First byte of multi-byte value.
	uint8_t				value[20];//currently max bytes is "NI" cmd with 20bytes length
} xbee_frame_remote_at_resp_t;

/// Useful typedef for casting either a local or remote response frame.
typedef union xbee_frame_at_response {
	/// First byte of header determines type (local or remote).
	uint8_t								frame_type;

	/// Use if .frame_type is #XBEE_FRAME_LOCAL_AT_RESPONSE.
	xbee_frame_local_at_resp_t		local;

	/// Use if .frame_type is #XBEE_FRAME_REMOTE_AT_RESPONSE.
	xbee_frame_remote_at_resp_t	remote;
} xbee_frame_at_response_t;

/*	@name
	Options for \c options field of xbee_header_transmit_t and
	xbee_header_transmit_explicit_t.
	@{
*/
/// XBee Transmit Option: Disable ACK [ZigBee and DigiMesh]
#define XBEE_TX_OPT_DISABLE_ACK				0x01
/// XBee Transmit Option: Disable Route Discovery [DigiMesh]
#define XBEE_TX_OPT_DISABLE_ROUTE_DISC		0x02
/// XBee Transmit Option: Enable Unicast NACK messages [DigiMesh]
#define XBEE_TX_OPT_ENABLE_UNICAST_NACK	0x04
/// XBee Transmit Option: Enable Unicast Trace Route messages [DigiMesh]
#define XBEE_TX_OPT_ENABLE_UNICAST_TRACE	0x08
/// XBee Transmit Option: Enable APS encryption (if EE=1) [ZigBee]
#define XBEE_TX_OPT_APS_ENCRYPT				0x20
/// XBee Transmit Option: Use extended timeout for this destination. [ZigBee]
#define XBEE_TX_OPT_EXTENDED_TIMEOUT		0x40
/// XBee Transmit Option: Point-Multipoint [DigiMesh]
#define XBEE_TX_OPT_MODE_POINT_MULTIPOINT	(1<<6)
/// XBee Transmit Option: Repeater mode (directed broadcast) [DigiMesh]
#define XBEE_TX_OPT_MODE_REPEATER			(2<<6)
/// XBee Transmit Option: DigiMesh (not available on 10k product) [DigiMesh]
#define XBEE_TX_OPT_MODE_DIGIMESH			(3<<6)
//@}

typedef struct xbee_frame_transmit_status_t {
	uint8_t			frame_type;			//< XBEE_FRAME_TRANSMIT_STATUS (0x8B)
	uint8_t			frame_id;
	uint16_t			network_address_be;
	uint8_t			retries;			// # of application Tx retries
	uint8_t			delivery;
/** @name
	Values for \c delivery member of xbee_frame_transmit_status_t.
	@{
*/
		/// XBee Transmit Delivery Status: Success [ZigBee, DigiMesh]
		#define XBEE_TX_DELIVERY_SUCCESS						0x00

		/// XBee Transmit Delivery Status: MAC ACK Failure [ZigBee]
		#define XBEE_TX_DELIVERY_MAC_ACK_FAIL				0x01

		/// XBee Transmit Delivery Status: CCA Failure [ZigBee]
		#define XBEE_TX_DELIVERY_CCA_FAIL					0x02

		/// XBee Transmit Delivery Status: LBT Failure [DigiMesh]
		#define XBEE_TX_DELIVERY_LBT_FAIL					0x02

		/// XBee Transmit Delivery Status: No Spectrum Available [DigiMesh]
		#define XBEE_TX_DELIVERY_NO_SPECTRUM				0x03

		/// XBee Transmit Delivery Status: Invalid Destination Endpoint [ZigBee, DigiMesh]
		#define XBEE_TX_DELIVERY_BAD_DEST_EP				0x15

		/// XBee Transmit Delivery Status: No Buffers [Smart Energy]
		#define XBEE_TX_DELIVERY_NO_BUFFERS					0x18

		/// XBee Transmit Delivery Status: Network ACK Failure [ZigBee, DigiMesh]
		#define XBEE_TX_DELIVERY_NET_ACK_FAIL				0x21

		/// XBee Transmit Delivery Status: Not Joined to Network [ZigBee]
		#define XBEE_TX_DELIVERY_NOT_JOINED					0x22

		/// XBee Transmit Delivery Status: Self-addressed [ZigBee]
		#define XBEE_TX_DELIVERY_SELF_ADDRESSED			0x23

		/// XBee Transmit Delivery Status: Address Not Found [ZigBee]
		#define XBEE_TX_DELIVERY_ADDR_NOT_FOUND			0x24

		/// XBee Transmit Delivery Status: Route Not Found [ZigBee, DigiMesh]
		#define XBEE_TX_DELIVERY_ROUTE_NOT_FOUND			0x25

		/// XBee Transmit Delivery Status: Relay of Broadcast not heard [ZigBee]
		#define XBEE_TX_DELIVERY_BROADCAST_NOT_HEARD		0x26

		/// XBee Transmit Delivery Status: Invalid Binding Table Index [ZigBee]
		#define XBEE_TX_DELIVERY_INVALID_BINDING_INDEX	0x2B

		/// XBee Transmit Delivery Status: Invalid Endpoint [ZigBee]
		#define XBEE_TX_DELIVERY_INVALID_EP					0x2C

		/// XBee Transmit Delivery Status: Attempted Broadcast with APS encryption
		/// [Smart Energy]
		#define XBEE_TX_DELIVERY_CANNOT_BROADCAST_APS	0x2D

		/// XBee Transmit Delivery Status: Attempted Unicast with APS encryption
		/// but EE=0 [Smart Energy]
		#define XBEE_TX_DELIVERY_ENCRYPTION_DISABLED		0x2E

		/// XBee Transmit Delivery Status: Resource error (lack of buffers,
		/// timers, etc.) [ZigBee]
		#define XBEE_TX_DELIVERY_RESOURCE_ERROR			0x32

		/// XBee Transmit Delivery Status: Data payload too large [ZigBee]
		#define XBEE_TX_DELIVERY_PAYLOAD_TOO_BIG			0x74

		/// XBee Transmit Delivery Status: Indirect message unrequested [ZigBee]
		#define XBEE_TX_DELIVERY_INDIRECT_NOT_REQ			0x75

		/// XBee Transmit Delivery Status: Key not authorized [Smart Energy]
		#define XBEE_TX_DELIVERY_KEY_NOT_AUTHORIZED		0xBB
//@}

	uint8_t			discovery;		// bitfield
/** @name
	Values for \c discovery member of xbee_frame_transmit_status_t.
	@{
*/
	/// XBee Transmit Discovery Status: No Discovery Overhead [ZigBee, DigiMesh]
		#define XBEE_TX_DISCOVERY_NONE				 0x00

	/// XBee Transmit Discovery Status: Address Discovery [ZigBee]
		#define XBEE_TX_DISCOVERY_ADDRESS			 0x01

	/// XBee Transmit Discovery Status: Route Discovery [ZigBee, DigiMesh]
		#define XBEE_TX_DISCOVERY_ROUTE				 0x02

	/// XBee Transmit Discovery Status: Extended Timeout Discovery [ZigBee]
		#define XBEE_TX_DISCOVERY_EXTENDED_TIMEOUT 0x40
//@}

} xbee_frame_transmit_status_t;

// ---- API for command lists ----
int xbee_handle_receive_explicit( uint8_t *rawframe, uint8_t  length );
int xbee_cmd_modem_status( uint8_t *rawframe, uint8_t  length );
int xbee_ATcmd_handle_response( uint8_t *rawframe, uint8_t  length );
int xbee_handle_transmit_status( uint8_t *rawframe, uint8_t  length );
#endif

  
