#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Register Addresses */
#define ALS_CONTR				0x80
#define ALS_MEAS_RATE 	0x85
#define PART_ID 				0x86
#define MANUFAC_ID 			0x87
#define ALS_DATA_CH1_0 	0x88
#define ALS_DATA_CH1_1 	0x89
#define ALS_DATA_CH0_0 	0x8A
#define ALS_DATA_CH0_1 	0x8B
#define ALS_STATUS 			0x8C


#define ALS_GAIN_1			0
#define ALS_GAIN_2			1
#define ALS_GAIN_4			2
#define ALS_GAIN_8			3
#define ALS_GAIN_48			6
#define ALS_GAIN_96			7


#define StandBy_Mode		0
#define Active_Mode			1

#define Integration_time_100ms		0
#define Integration_time_50ms			1
#define Integration_time_200ms		2
#define Integration_time_400ms		3
#define Integration_time_150ms		4
#define Integration_time_250ms		5
#define Integration_time_300ms		6
#define Integration_time_350ms		7


#define Measuement_Time_50ms			0
#define Measuement_Time_100ms			1
#define Measuement_Time_200ms			2
#define Measuement_Time_500ms			3
#define Measuement_Time_1000ms		4
#define Measuement_Time_2000ms		5
