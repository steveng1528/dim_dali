#ifndef _DEBUG_H
#define _DEBUG_H

/* ------------------------------------------------------------------------------
 *         Headers
 * ----------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdint.h>

#define log_debug(fmt, ...)\
        do { printf("[DEBUG] %10s:%10d:%15s(): " fmt, __FILE__, __LINE__, __func__,## __VA_ARGS__); } while (0)

#define log_err(fmt, ...) \
        do { printf( "[ERROR] %10s:%10d:%15s(): " fmt, __FILE__,  __LINE__, __func__,## __VA_ARGS__); } while (0)


#define log_warn(fmt, ...) \
        do { printf("[WARN] %10s:%10d:%15s(): " fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)

#define log_info(fmt, ...) \
        do { printf( "[INFO] %10s:%10d:%15s(): " fmt, __FILE__, \
                                __LINE__, __func__, ##__VA_ARGS__); } while (0)



	
#endif /* _DEBUG_H_ */


uint8_t u8_Debug_Printf_Uart_Init(void);
