/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Remote_Controller.h"
#include "Move_Controlor.h"
#include "CAN_Operation.h"
#include "PID_Controlor.h"
#include "math.h"
#include "Usart_Printf.h"
#include "Location_Module_Recieve.h"
#include "Navi_Control.h"
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
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int Start_Fixed = 0;
float Now_Pos_X, Now_Pos_Y;

void Rough_Deadbond()
{
	X_PID.deadbond = 0.2f;
	Y_PID.deadbond = 0.2f;
	Angle_PID.deadbond = 2.0f;
}

void Precise_Deadbond()
{
	X_PID.deadbond = 0.01f;
	Y_PID.deadbond = 0.01f;
	Angle_PID.deadbond = 5.0f;
}

void High_Speed()
{
	X_PID.max_output = 4000;
	Y_PID.max_output = 4000;
}

void Medium_Speed()
{
	X_PID.max_output = 2000;
	Y_PID.max_output = 2000;
}

void Point_Running_Task(float Set_X,float Set_Y,float Set_Angle)
{
	while ((fabs(Set_X-Location_Data.Pos_X) > X_PID.deadbond) || (fabs(Set_Y - Location_Data.Pos_Y) > Y_PID.deadbond) ||
				 (fabs(Set_Angle-Location_Data.Angle) > Angle_PID.deadbond))
	{
		if (Remote_Controller_Data.Remote_Controller.s[0] == 3){
			MecanumChassis_OmniDrive(0,0,0);
		}else if (Remote_Controller_Data.Remote_Controller.s[0] == 2){
			Remote_Moving_Control();
			Start_Fixed = 1;
			Now_Pos_X = Location_Data.Pos_X;
			Now_Pos_X = Location_Data.Pos_Y;
		}
		else{
			if (Start_Fixed){
				Fixed_Pos_X += Location_Data.Pos_X - Now_Pos_X;
				Fixed_Pos_Y += Location_Data.Pos_Y - Now_Pos_Y;
				Start_Fixed = 0;
			}
			Point_Navi_Move(Set_X,Set_Y,Set_Angle);
		}
	}
}

void Wait_Task(int time)
{
	for (int i=1;i<=(time/10);i++){
		MecanumChassis_OmniDrive(0,0,0);
	}
	X_PID.kp_output = 0;
	X_PID.ki_output = 0;
	X_PID.kd_output = 0;
	X_PID.err_sum = 0;
	X_PID.output = 0;
	Y_PID.kp_output = 0;
	Y_PID.ki_output = 0;
	Y_PID.kd_output = 0;
	Y_PID.err_sum = 0;
	Y_PID.output = 0;
	Angle_PID.kp_output = 0;
	Angle_PID.ki_output = 0;
	Angle_PID.kd_output = 0;
	Angle_PID.err_sum = 0;
	Angle_PID.output = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Base_Control_Init();
  Remote_Controller_Init();
	Locator_Rx_Init();
	CAN2_Filter_Init();
	Navi_Control_Init();
	Usart1_TX_DMA_Init();
	float Dead_Bond = 0.0001f;
	float Angle = Location_Data.Angle;
	start_locator = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	float x = Location_Data.Pos_X;
//	float y = Location_Data.Pos_Y;
//	while ((Set_Y-y) > Dead_Bond)
//	{
//		y = Location_Data.Pos_Y;
//		if (start_Remote && start_locator){
//			MecanumChassis_OmniDrive(0,200,0);
//		}else{
//			MecanumChassis_OmniDrive(0,0,0);
//		}
//		if (Remote_Controller_Data.Remote_Controller.s[0] == 1) start_Remote = 1;
//		else start_Remote = 0;
//	}
	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(Bee_GPIO_Port,Bee_Pin,GPIO_PIN_RESET);
//	while ((Set_Angle-Angle) > 1.0f)
//	{
//		Angle = Location_Data.Angle;
//		if (start_Remote && start_locator){
//			MecanumChassis_OmniDrive(0,0,5);
//		}else{
//			MecanumChassis_OmniDrive(0,0,0);
//		}
//		if (Remote_Controller_Data.Remote_Controller.s[0] == 1) start_Remote = 1;
//		else start_Remote = 0;
////	}
	while (start_locator < 1)
	{
		MecanumChassis_OmniDrive(0,0,0);
	}
	HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
	Rough_Deadbond();
	Point_Running_Task(0,3.3f,0);
	Medium_Speed();
	Point_Running_Task(-2.7f,3.3f,0);
	High_Speed();
	Precise_Deadbond();
	Point_Running_Task(-2.7f,1.41f,0);
	Wait_Task(1000);
	Rough_Deadbond();
	Point_Running_Task(-2.7f,3.3f,0);
	Precise_Deadbond();
	Point_Running_Task(-0.57f,3.5f,92);
	Wait_Task(1000);

	Rough_Deadbond();
	Point_Running_Task(-3.0f,3.3f,0);
	Precise_Deadbond();
	Point_Running_Task(-3.0f,1.41f,0);
	Wait_Task(1000);
	Rough_Deadbond();
	Point_Running_Task(-3.0f,3.3f,0);
	Precise_Deadbond();
	Point_Running_Task(-0.57f,3.5f,92);
	Wait_Task(1000);

	Rough_Deadbond();
	Point_Running_Task(-3.3f,3.3f,0);
	Precise_Deadbond();
	Point_Running_Task(-3.3f,1.41f,0);
	Wait_Task(1000);
	Rough_Deadbond();
	Point_Running_Task(-3.3f,3.3f,0);
	Precise_Deadbond();
	Point_Running_Task(-0.57f,3.5f,92);
	Wait_Task(1000);

	Rough_Deadbond();
	Point_Running_Task(-3.6f,3.3f,0);
	Precise_Deadbond();
	Point_Running_Task(-3.6f,1.41f,0);
	Wait_Task(1000);
	Rough_Deadbond();
	Point_Running_Task(-3.6f,3.3f,0);
	Precise_Deadbond();
	Point_Running_Task(-0.57f,3.5f,92);
	Wait_Task(1000);

	Point_Running_Task(-5.3f,3.8f,-90);
	Wait_Task(1000);
	Point_Running_Task(-0.57f,3.5f,92);
	Wait_Task(1000);
	Rough_Deadbond();
	Point_Running_Task(0,3.3f,0);
	Precise_Deadbond();
	Point_Running_Task(0,0,0);
	Wait_Task(1000);
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		if (start_locator > 1) HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
//		else HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
		//Remote_Moving_Control();
		//if (Remote_Controller_Data.Remote_Controller.ch[4] != 0) Usart_Printf("ACT0");
		if (Remote_Controller_Data.Remote_Controller.s[0] == 1){
			//Gyro_Mode_Drive(0,1000,60);
			//MecanumChassis_OmniDrive(0,0,0);
//			if (Remote_Controller_Data.Remote_Controller.s[1] == 2){
//				Point_Navi_Move(0,0,180);
//			}else if (Remote_Controller_Data.Remote_Controller.s[1] == 3){
//				Point_Navi_Move(0,0,0);
//			}else if (Remote_Controller_Data.Remote_Controller.s[1] == 1){
//				Point_Navi_Move(0,2.0f,0);
//			}
			//Point_Navi_Move(0,0,180);
		}else if (Remote_Controller_Data.Remote_Controller.s[0] == 3){
			MecanumChassis_OmniDrive(0,0,0);
			if (Remote_Controller_Data.Remote_Controller.s[1] == 3){
				HAL_Delay(10);
				if (Remote_Controller_Data.Remote_Controller.s[1] == 2){
					Usart_Printf("ACT0");
					HAL_Delay(10);
				}
			}
		}
		else if (Remote_Controller_Data.Remote_Controller.s[0] == 2) Remote_Moving_Control();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  __disable_irq();
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

