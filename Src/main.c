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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

#define SONY_HDR_MARK             2400
#define SONY_HDR_SPACE             600
#define SONY_ONE_MARK             1200
#define SONY_ZERO_MARK             600

int  MATCH_MARK (int measured_ticks,  int desired_us);
int  MATCH_SPACE (int measured_ticks,  int desired_us);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t i = 0;
	uint16_t raw_data_index_over_500[15] = {0};			//all is 12bit plus frame start 1 equal 13
	uint16_t raw_data_index_over_500_sum = 0;				//
	uint16_t raw_data[30] = {0};										//
	uint16_t raw_data_index_new = 0;		
	uint16_t off_set = 0;
	uint16_t ir_data = 0;
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
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart1,"hello",6,0xFF);
	
	HAL_Delay(300);
	if( SEND_IR_DATA ){
		//TIM2 pwm   CH1-PA5-PWM3 CH2-PA1-PWM1 CH4-PA3-PWM2
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		//TIM4 CH1-PB6-PWM4
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		//TIM10 CH1-PB8-PWM5
		//HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
		
		//TIM2->CCR1 = 45;		//PA5 TIM2-CH1 pwm3
		
		//oc compare  output sony data
		HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
		
		//input capture direct
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
	}
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if( SEND_IR_DATA == 0 )
		{
			HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
		}
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,0);
		HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,0);
		HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,0);
		HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,0);
		HAL_GPIO_WritePin(LED12_GPIO_Port,LED12_Pin,0);
		HAL_GPIO_WritePin(LED13_GPIO_Port,LED13_Pin,0);
		HAL_Delay(50);
		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,1);
		HAL_GPIO_WritePin(LED6_GPIO_Port,LED6_Pin,1);
		HAL_GPIO_WritePin(LED7_GPIO_Port,LED7_Pin,1);
		HAL_GPIO_WritePin(LED8_GPIO_Port,LED8_Pin,1);
		HAL_GPIO_WritePin(LED12_GPIO_Port,LED12_Pin,1);
		HAL_GPIO_WritePin(LED13_GPIO_Port,LED13_Pin,1);
		HAL_Delay(300);
		if( rc_in_start )
		{
			HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_4);
			rc_in_start = 0;
			if( rc_in[0] > 20000 )	//20000us=20ms
			{
				printf("rec data frame,count is %d\r\n", tim3_channel1_count);
//				for( i = 0; i < tim3_channel1_count; i++ )			//find pulse over 500 us
//				{
//					printf("rc_in[%d] is : %d\r\n",i,rc_in[i]);
//				}
//				printf("\r\n");
				raw_data_index_over_500_sum = 0;
				for( i = 0; i < tim3_channel1_count; i++ )			//find pulse over 500 us
				{
						if( rc_in[i] > 500 )		//
						{
							raw_data_index_over_500[raw_data_index_over_500_sum] = i;
							//printf("raw_data_index_over_500[%d] is %d\r\n",raw_data_index_over_500_sum,raw_data_index_over_500[raw_data_index_over_500_sum]);
							raw_data_index_over_500_sum++;
						}
				}
				if( raw_data_index_over_500_sum != 13)
				{
					printf("data is wrong!!!!!!!\r\n");
				}
				else
				{
					//decode into raw_data
					for( i = 0; i < 13; i++ )
					{
						raw_data[2*i] = (raw_data_index_over_500[i+1] - raw_data_index_over_500[i]) * 200;
						raw_data[2*i+1] = rc_in[raw_data_index_over_500[i+1]];
					}
					//decode into ir_data
					if(!MATCH_MARK(raw_data[off_set++],SONY_HDR_MARK))
					{
						printf("data error\r\n");
					}
					while( off_set+1 < 25){
						if(!MATCH_SPACE(raw_data[off_set++], SONY_HDR_SPACE)) 
							break;
						
						if			(MATCH_MARK(raw_data[off_set],SONY_ONE_MARK)) 
							ir_data = (ir_data << 1) | 1;
						else if	(MATCH_MARK(raw_data[off_set],SONY_ZERO_MARK)) 
							ir_data = (ir_data << 1) | 0;
						else
								break;
						off_set++;
					}
					ir_data = (ir_data << 1) | 0;
					printf("ir_data is 0x%x",ir_data);
					
				}
			}
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



// Upper and Lower percentage tolerances in measurements
#define TOLERANCE       0.2



int  MATCH_MARK (int measured_ticks,  int desired_us)
{
	int passed = ((measured_ticks >=  (desired_us)*(1-TOLERANCE))
                && (measured_ticks <= (desired_us)*(1+TOLERANCE)));
 	return passed;
}

int  MATCH_SPACE (int measured_ticks,  int desired_us)
{
  int passed = ((measured_ticks >= (desired_us)*(1-TOLERANCE))
                && (measured_ticks <= (desired_us)*(1+TOLERANCE)));
 	return passed;
}



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
