
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"
#include "stm32l4xx_hal_can.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

//CAN_RxHeaderTypeDef rxHeader;
//uint8_t rxMessage[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_CAN1_Init(4,CAN_BS1_12TQ,CAN_BS1_5TQ);

  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

  HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, HAL_CAN_TxMailbox0CompleteCallback);
  HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, HAL_CAN_RxFifo0MsgPendingCallback);
  HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, HAL_CAN_RxFifo1MsgPendingCallback);

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   CAN_TxHeaderTypeDef TxHeader;
  	  TxHeader.StdId = 0x321;
  	  TxHeader.RTR = CAN_RTR_DATA;
  	  TxHeader.IDE = CAN_ID_STD;
  	  TxHeader.DLC = 2;
  	uint8_t txData[8] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08} ;

  	CAN_Baudrate baud = BAUD_500K;

  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
	  //HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxMessage);
	  //HAL_Delay(100);
	  //if(MX_CAN1_Init(4,CAN_BS1_12TQ,CAN_BS1_5TQ) == HAL_OK )
      switch (baud)
      {
		  case BAUD_500K:
			  HAL_CAN_DeInit(&hcan1);
			  MX_CAN1_Init(4, CAN_BS1_12TQ, CAN_BS1_5TQ);
			  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
			  HAL_Delay(10);

			  if(is_tx_completed)
			  {
				is_tx_completed = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				// yesil ledi yak
				baud = BAUD_DEFAULT;
			  }
			  else
			  {
			    baud = BAUD_250K;
			  }
			 break;
		  case BAUD_250K:
			  HAL_CAN_DeInit(&hcan1);
			  MX_CAN1_Init(8,CAN_BS1_12TQ,CAN_BS1_5TQ);
			  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
			  HAL_Delay(10);

			          if(is_tx_completed)
			          {
			          is_tx_completed = 0;
			          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			          // yesil ledi yak
				      baud = BAUD_DEFAULT;
			          }
			          else
			          {
				      baud = BAUD_1M;
			          }
			          break;
		  case BAUD_1M:
			          HAL_CAN_DeInit(&hcan1);
			          // if(MX_CAN1_Init(2,CAN_BS1_12TQ,CAN_BS1_5TQ))
			          MX_CAN1_Init(2,CAN_BS1_12TQ,CAN_BS1_5TQ);
			          HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
			          HAL_Delay(10);
			          if(is_tx_completed)
			          {
			        	is_tx_completed = 0;
			        	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			        	// yesil ledi yak
		  				baud = BAUD_DEFAULT;
		  			  }
		  			  else
		  			  {
		  				baud = BAUD_400K;
		  			  }
		  			  break;

		  case BAUD_400K:
			         HAL_CAN_DeInit(&hcan1);
		  		  	 MX_CAN1_Init(5,CAN_BS1_12TQ,CAN_BS1_5TQ);
		  		  	 HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
		  		  	 HAL_Delay(10);
		  		  	 if(is_tx_completed)
		  		  	  {
		  		  		is_tx_completed = 0;
		  		  	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  		  		// yesil ledi yak
		  		  		baud = BAUD_DEFAULT;
		  		  	  }
		  		  	  else
		  		  	  {
		  		  		baud = BAUD_200K;
		  		  	  }
		  		  	  break;
		  case BAUD_200K:
			  HAL_CAN_DeInit(&hcan1);
		  	  MX_CAN1_Init(10,CAN_BS1_12TQ,CAN_BS1_5TQ);
		  	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
		  	  HAL_Delay(10);
		  	  if(is_tx_completed)
		  		  	  {
		  		        is_tx_completed = 0;
		  		        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  		  		// yesil ledi yak
		  		  		baud = BAUD_DEFAULT;
		  		  	  }
		  		  	  else
		  		  	  {
		  		  		baud = BAUD_125K;
		  		  	  }
		  		  	  break;
		  case BAUD_125K:
			  HAL_CAN_DeInit(&hcan1);
		 	  MX_CAN1_Init(16,CAN_BS1_12TQ,CAN_BS1_5TQ);
		 	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
		 	  HAL_Delay(10);
		 	  if(is_tx_completed)
		 		  	  {
		 		        is_tx_completed = 0;
		 		        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		 		  		// yesil ledi yak
		 		  		baud = BAUD_DEFAULT;
		 		  	  }
		 		  	  else
		 		  	  {
		 		  		baud = BAUD_100K;
		 		  	  }
		 		  	  break;

		  case BAUD_100K:
			  HAL_CAN_DeInit(&hcan1);
		  	  MX_CAN1_Init(20,CAN_BS1_12TQ,CAN_BS1_5TQ);
		  	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
		  	  HAL_Delay(10);
		  	  if(is_tx_completed)
		  		      {
		  		        is_tx_completed = 0;
		  		        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  		 		// yesil ledi yak
		  		 	    baud = BAUD_DEFAULT;
		  		 	  }
		  		 	  else
		  		 	  {
		  		 		baud = BAUD_50K;
		  		 	  }
		  		 	  break;

		  case BAUD_50K:
			  HAL_CAN_DeInit(&hcan1);
		      MX_CAN1_Init(40,CAN_BS1_12TQ,CAN_BS1_5TQ);
		      HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
		      HAL_Delay(10);
		      if(is_tx_completed)
		 		  	  {
		    	         is_tx_completed = 0;
		 		  		 // yesil ledi yak
		    	         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		 		  		baud = BAUD_DEFAULT;
		 		  	  }
		 		  	  else
		 		  	  {
		 		  		 baud = BAUD_25K;
		 		  	  }
		 		  	  break;

		  case BAUD_25K:
			  HAL_CAN_DeInit(&hcan1);
		  		 	  MX_CAN1_Init(80,CAN_BS1_12TQ,CAN_BS1_5TQ);
		  		 	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
		  		 	  HAL_Delay(10);
		  		 	  if(is_tx_completed)
		  		 	  {
		  		 		 is_tx_completed = 0;
		  		 		 // yesil ledi yak
		  		 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  		 	  baud = BAUD_DEFAULT;
		  		 	  }
		  		 	  else
		  		 	  {
		  		 		baud = BAUD_10K;
		  		 	  }
		  		 	  break;
		  case BAUD_10K:
			  HAL_CAN_DeInit(&hcan1);
		  		  	  MX_CAN1_Init(200,CAN_BS1_12TQ,CAN_BS1_5TQ);
		  		  	  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, txData, (uint32_t *)CAN_TX_MAILBOX0);
		  		  	  HAL_Delay(10);
		  		  	  if(is_tx_completed)
		  		  	  {
		  		  		 is_tx_completed = 0;
		  		  		 // yesil ledi yak
		  		  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		  		  		baud = BAUD_DEFAULT;
		  		  	  }
		  		  		 else
		  		  	  {
		  		  		  baud = BAUD_500K;
		  		  	  }
		  		  	   break;
		  case BAUD_DEFAULT:
		  default:
			  break;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
