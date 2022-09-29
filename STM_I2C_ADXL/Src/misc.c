#include "misc.h"

 void vprint(const char *fmt,va_list argp){
	char string[200];
	if(0<vsprintf(string,fmt,argp)){
	
		HAL_UART_Transmit(&hlpuart1,(uint8_t*)string,strlen(string),0xFFFFFF);
		
	}
}
void debug_printf(const char *fmt, ...){
	va_list argp;
	va_start(argp,fmt);
	vprint(fmt,argp);
	va_end(argp);

}

