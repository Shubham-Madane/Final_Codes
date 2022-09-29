/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WriteAddr		0x52
#define ReadAddr		0x53

#define ALS_CONTR 			0x80
#define ALS_MEAS_RATE		0x85
#define PART_ID 				0x86
#define MANUFAC_ID			0x87
#define ALS_DATA_CH1_0 	0x88
#define ALS_DATA_CH1_1	0x89
#define ALS_DATA_CH0_0 	0x8A
#define ALS_DATA_CH0_1	0x8B
#define ALS_STATUS 			0x8C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t Wbuffer[6]={0},Rbuffer[6]={0},Part_ID,Manuf_ID,Gain,Int_Time;
	uint16_t Ch1_data,Ch0_data;
	float ratio,Lux_value,Int_Factor;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	
		if(HAL_I2C_IsDeviceReady(&hi2c1,0x52,5,100)==HAL_OK){			//check if sensor is responding back
		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
		HAL_Delay(500);
			HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
		HAL_Delay(500);
			HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);
		HAL_Delay(500);
			HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET);
		}
		
		HAL_Delay(100);							//startup time of 100ms
		Wbuffer[0]=ALS_CONTR;
		Wbuffer[1]=0x09;  //00000001		//0000 1001
		
		//HAL_I2C_Mem_Write(&hi2c1,WriteAddr,ALS_CONTR,1,&Wbuffer[1],1,100);
		
		HAL_I2C_Master_Transmit(&hi2c1,WriteAddr,Wbuffer,2,100);		//Set the sensor to active mode
		memset(Wbuffer,0,6);
		
		HAL_Delay(10);						//Wakeup time form standby mode
		
		Wbuffer[0]=ALS_CONTR;
		
		//HAL_I2C_Mem_Read(&hi2c1,ReadAddr,ALS_CONTR,1,Rbuffer,1,100);
		
		HAL_I2C_Master_Transmit(&hi2c1,WriteAddr,Wbuffer,1,100);		//Read the gain value
		HAL_I2C_Master_Receive(&hi2c1,ReadAddr,Rbuffer,1,100);
		
		Gain = (Rbuffer[0]>>2)&0x07; // 00000111	00000100  :  00000011 :00000011	
		//0010100 >>2		00100101 00000111		00000101
		switch(Gain)
		{
			case 0: Gain = 1;  break;
			case 1: Gain = 2;  break;
			case 2: Gain = 4;  break;
			case 3: Gain = 8;  break;
			case 6: Gain = 48;  break;
			case 7: Gain = 96;  break;
		}
		debug_printf("\n\rGain:%d",Gain);
		
		memset(Wbuffer,0,6);
		memset(Rbuffer,0,6);
		
		Wbuffer[0] = PART_ID;
		HAL_I2C_Master_Transmit(&hi2c1,WriteAddr,Wbuffer,1,100);		//Read the part ID
		HAL_I2C_Master_Receive(&hi2c1,ReadAddr,Rbuffer,1,100);
		
		Part_ID = (Rbuffer[0]>>4)&0x0F;
		debug_printf("\n\rPart_ID:%d",Part_ID);
		
		memset(Wbuffer,0,6);
		memset(Rbuffer,0,6);
		
		Wbuffer[0] = MANUFAC_ID;
		HAL_I2C_Master_Transmit(&hi2c1,WriteAddr,Wbuffer,1,100);		//Read the manufacture ID
		HAL_I2C_Master_Receive(&hi2c1,ReadAddr,Rbuffer,1,100);
		
		Manuf_ID = Rbuffer[0];
		debug_printf("\n\rManuf_ID:%d",Manuf_ID);
		
		memset(Wbuffer,0,6);
		memset(Rbuffer,0,6);
		
		Wbuffer[0]=ALS_MEAS_RATE;
		HAL_I2C_Master_Transmit(&hi2c1,WriteAddr,Wbuffer,1,100);		//Read the integration time
		HAL_I2C_Master_Receive(&hi2c1,ReadAddr,Rbuffer,1,100);
		
		Int_Time = (Rbuffer[0]>>3)&0x07;
		switch(Int_Time)
		{
			case 0: Int_Factor = 1;  break;
			case 1: Int_Factor = 0.5;  break;
			case 2: Int_Factor = 2;  break;
			case 3: Int_Factor = 4;  break;
			case 4: Int_Factor = 1.5;  break;
			case 5: Int_Factor = 2.5;  break;
			case 6: Int_Factor = 3;  break;
			case 7: Int_Factor = 3.5;  break;
		}
		debug_printf("\n\rInt_Factor:%f",Int_Factor);
		
		memset(Wbuffer,0,6);
		memset(Rbuffer,0,6);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		
		
		Wbuffer[0]=ALS_DATA_CH1_0;
		HAL_I2C_Master_Transmit(&hi2c1,0x52,Wbuffer,1,100);			//Read Data_Reg
		HAL_I2C_Master_Receive(&hi2c1,0x53,&Rbuffer[0],4,100);
		
		Ch1_data = (Rbuffer[1]<<8)+(Rbuffer[0]);
		Ch0_data = (Rbuffer[3]<<8)+(Rbuffer[2]);
		
		ratio = Ch1_data/(Ch1_data+Ch0_data);
		//conversion of raw data to lux values as giver in Appendix A of LTR-329ALS-01
		if(ratio<0.45)
			Lux_value = ((1.7743*Ch0_data)+(1.1059*Ch1_data))/Gain/Int_Factor;
		else if(ratio<0.64&&ratio>=0.45)
			Lux_value = ((4.2785*Ch0_data)-(1.9548*Ch1_data))/Gain/Int_Factor;
		else if(ratio<0.85&&ratio>=0.64)
			Lux_value = ((0.5926*Ch0_data)+(0.1185*Ch1_data))/Gain/Int_Factor;
		 else 
			Lux_value = 0;
		
		debug_printf("\n\rLux Value(in LUX):%f",Lux_value);
		
		HAL_Delay(2000);		//Delay of 2 seconds between 2 measurements
			
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
