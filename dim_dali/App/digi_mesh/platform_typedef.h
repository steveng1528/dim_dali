/*
 * Copyright (c) 2010-2012 Digi International Inc.,
 * All rights not expressly granted are reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Digi International Inc. 11001 Bren Road East, Minnetonka, MN 55343
 * =======================================================================
 */

/**
	@addtogroup hal_dos
	@{
	@file platform_typedef.h
	Header for DOS platform (using Watcom C cross compiler).

	This file is automatically included by xbee/platform.h.

*/
#ifndef __XBEE_PLATFORM_DOS
#define __XBEE_PLATFORM_DOS

    // macro used to declare a packed structure (no alignment of elements)
    #define PACKED_STRUCT		__packed struct

    #define _f_memcpy		memcpy
    #define _f_memset		memset
/*
    typedef signed char		int8_t;
    typedef unsigned char	uint8_t;
    typedef short				int16_t;
    typedef unsigned short	uint16_t;
*/
    #define __FUNCTION__ 	""

    // This type isn't in stdint.h
    typedef int					bool_t;

/// Typedef used to hold a 64-bit IEEE address, represented as 8 bytes,
/// 4 16-bit values or 2 32-bit values.
/// Note that (for now) all addr64 elements are stored MSB-first (the order
/// used in XBee frames).
/// @todo update all addr64 variables and structure elements to end in _be
/// (big-endian) or _le (little-endian) where appropriate.  Add functions
/// to convert 64-bit values between host byte order and big/little endian.
typedef union {
	uint8_t			b[8];
	uint16_t			u[4];
	uint32_t			l[2];
} addr64;


#ifndef ADDR64_FORMAT_SEPARATOR
	/// Separator used by addr64_format(), defaults to '-' unless specified
	/// at compile time.
	#define ADDR64_FORMAT_SEPARATOR '-'
#endif

/// Size of character buffer to pass to addr64_format()
/// (8 2-character bytes, 7 separators and 1 null).
#define ADDR64_STRING_LENGTH (8 * 2 + 7 + 1)

/** @name Reserved/Special WPAN network (16-bit) addresses
	@{
*/
/// network broadcast address for all nodes
#define WPAN_NET_ADDR_BCAST_ALL_NODES	0xFFFF
/// network broadcast address for non-sleeping devices
#define WPAN_NET_ADDR_BCAST_NOT_ASLEEP	0xFFFD
/// network broadcast address for all routers (and coordinators)
#define WPAN_NET_ADDR_BCAST_ROUTERS		0xFFFC

/// used to indicate 64-bit addressing (16-bit address is ignored)
#define WPAN_NET_ADDR_UNDEFINED			0xFFFE

/// network coordinator always uses network address 0x0000
#define WPAN_NET_ADDR_COORDINATOR		0x0000
//@}

/**
	@name
	These error names are used throughout the library.  Some platforms don't
	define them in errno.h, so we define them here using arbitrary values.
	@{

	@def E2BIG
		argument list too long (POSIX.1)
	@def EACCES
		permission denied (POSIX.1)
	@def EAGAIN
		resource temporarily unavailable (POSIX.1)
	@def EBADMSG
		bad message (POSIX.1)
	@def EBUSY
		device or resource busy (POSIX.1)
	@def ECANCELED
		operation canceled (POSIX.1)
	@def EEXIST
		file exists (POSIX.1)
	@def EILSEQ
		illegal byte sequence (POSIX.1, C99)
	@def EINVAL
		invalid argument (POSIX.1)
	@def EIO
		input/output error (POSIX.1)
	@def EMSGSIZE
		message too long (POSIX.1)
	@def ENODATA
		no message is available on the STREAM head read queue (POSIX.1)
	@def ENOENT
		no such file or directory (POSIX.1)
	@def ENOSPC
		no space left on device (POSIX.1)
	@def ENOSYS
		function not implemented (POSIX.1)
	@def ENOTSUP
		operation not supported (POSIX.1)
	@def EPERM
		operation not permitted (POSIX.1)
	@def ETIMEDOUT
		connection timed out (POSIX.1)
*/
#ifndef ENODATA
	#define ENODATA	20000
#endif
#ifndef EINVAL
	#define EINVAL		20001
#endif
#ifndef EIO
	#define EIO			20002
#endif
#ifndef EBUSY
	#define EBUSY		20003
#endif
#ifndef EEXIST
	#define EEXIST		20004
#endif
#ifndef ENOSPC
	#define ENOSPC		20005
#endif
#ifndef ENOENT
	#define ENOENT		20006
#endif
#ifndef E2BIG
	#define E2BIG		20007
#endif
#ifndef EBADMSG
	#define EBADMSG	20010
#endif
#ifndef ENOTSUP
	#define ENOTSUP	20011
#endif
#ifndef ETIMEDOUT
	#define ETIMEDOUT	20012
#endif
#ifndef EILSEQ
	#define EILSEQ		20013
#endif
#ifndef EAGAIN
	#define EAGAIN		20014
#endif
#ifndef ENOSYS
	#define ENOSYS		20015
#endif
#ifndef EACCES
	#define EACCES		20016
#endif
#ifndef ECANCELED
	#define ECANCELED	20017
#endif
#ifndef EMSGSIZE
	#define EMSGSIZE	20018
#endif
#ifndef EPERM
	#define EPERM		20019
#endif

/// Single structure to hold an 802.15.4 device's 64-bit IEEE/MAC address
/// and 16-bit network address.
typedef struct _wpan_address_t {
	addr64		ieee;
	uint16_t		network;
} wpan_address_t;

#endif		// __XBEE_PLATFORM_DOS

