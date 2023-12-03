/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "smhome_proto.h"

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

uint16_t *idBase0;
uint16_t *idBase1;
uint32_t *idBase2;
uint32_t *idBase3;

uint8_t THIS_CANID = 0;

bool doPolling = false;

uint8_t CAN_Masters[CAN_MASTERS_NUM] = {0,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void sleepMode();
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
	#define LOCAL_UID_BASE 0x1FFFF7E8

	idBase0 = (uint16_t*)(LOCAL_UID_BASE);
	idBase1 = (uint16_t*)(LOCAL_UID_BASE + 0x02);
	idBase2 = (uint32_t*)(LOCAL_UID_BASE + 0x04);
	idBase3 = (uint32_t*)(LOCAL_UID_BASE + 0x08);

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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */


//	println("Init done.");
//	println("UID: %4x-%4x-%8lx-%8lx", *idBase0, *idBase1, *idBase2, *idBase3);


	//***   Read CAN address
	THIS_CANID = 0x0;
	//  Set can master addr to broadcast
	CAN_Masters[0] = 0xFF;

	if ( HAL_GPIO_ReadPin(GPIOB, ADDR_B1_Pin) == GPIO_PIN_RESET) {
		THIS_CANID = THIS_CANID | 1;
	}
	if ( HAL_GPIO_ReadPin(GPIOA, ADDR_B2_Pin) == GPIO_PIN_RESET) {
		THIS_CANID = THIS_CANID | (1 << 1);
	}
	if ( HAL_GPIO_ReadPin(GPIOA, ADDR_B3_Pin) == GPIO_PIN_RESET) {
		THIS_CANID = THIS_CANID | (1 << 2);
	}
	if ( HAL_GPIO_ReadPin(GPIOA, ADDR_B4_Pin) == GPIO_PIN_RESET) {
		THIS_CANID = THIS_CANID | (1 << 3);
	}
	if ( HAL_GPIO_ReadPin(GPIOB, ADDR_B5_Pin) == GPIO_PIN_RESET) {
		THIS_CANID = THIS_CANID | (1 << 4);
	}
	if ( HAL_GPIO_ReadPin(GPIOB, ADDR_B6_Pin) == GPIO_PIN_RESET) {
		THIS_CANID = THIS_CANID | (1 << 5);
	}
	if ( HAL_GPIO_ReadPin(GPIOB, ADDR_B7_Pin) == GPIO_PIN_RESET) {
		THIS_CANID = THIS_CANID | (1 << 6);
	}
	if ( HAL_GPIO_ReadPin(GPIOB, ADDR_B8_Pin) == GPIO_PIN_RESET) {
		THIS_CANID = THIS_CANID | (1 << 7);;
	}


	//  Sensors DB Init
	Sensor_DB_Init();
	q_Init(CAN_RX_STCK_SIZE);

	// CAN interface start
	//  See https://www.youtube.com/watch?v=KHNRftBa1Vc&list=PLGJHXpiemDq2U8F76MoQS7IdQG_yxS1WB&index=24
	HAL_CAN_Start(&hcan);
	CAN_ListenFilter_Init(&hcan, THIS_CANID);
	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_ADCEx_Calibration_Start(&hadc1);

	SMH_ADC_RunConversation();

	//TODO: Load saved configuration.

	// https://www.micropeta.com/video62
	// Start timer for 1 sec. interrupt.
	HAL_TIM_Base_Start_IT(&htim2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	sleepMode();

	while (!q_isEmpty()) {
		CanPacket* pkt = q_Pop();
		SMHome_InputSelector(pkt);
		q_ClearPkt(pkt);
	}

	if (doPolling) {
		SMH_SensorDOPolling();
		doPolling = false;
	}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/**
  * @brief  Put CPU in sleep mode
  * @param  None
  * @retval None
  */
void sleepMode() {

#ifdef DEBUG_PRINT_UART
	HAL_UART_DeInit(&huart2);
#endif

	/* Stopping  Systick */
	HAL_SuspendTick();

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnterSLEEPMode(0, PWR_SLEEPENTRY_WFE);

	/* Starting Systick */
	HAL_ResumeTick();

#ifdef DEBUG_PRINT_UART
	MX_USART2_UART_Init();
#endif
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
  __disable_irq();
//  println(" Global error. Halt.");
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//	println("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
