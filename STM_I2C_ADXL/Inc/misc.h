/* File misc.  */
#ifndef MISC
#define MISC

#include "usart.h"
#include "stm32g4xx_hal.h"
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

typedef enum{
	SUCESS,
	FAIL = 1,
	TIMEOUT=2,
	BUSY=3
}ORET_STATUS;
 
void debug_printf(const char *fmt, ...);
void vprint(const char *fmt,va_list argp);

#endif 
#define DEBUG 1
		#if DEBUG
				#define DEBUG_LOG(LOG_MESSAGE)  \
															debug_printf("\n\rDEBUG_LOG : ");\
															debug_printf(LOG_MESSAGE);\
															
		#else 
				#define DEBUG_LOG(LOG_MESSAGE)  \
															
		
#endif

