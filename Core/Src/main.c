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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280_defs.h"
#include "bmp280.h"
#include "stdio.h"
#include "arm_math.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "LCD.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BMP280_SPI (&hspi4)
#define BMP280_CS1 1
#define BMP280_CS2 2
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_PARAM_KP        20
#define PID_PARAM_KI        0.002
#define PID_PARAM_KD       	0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int8_t bmp280_spi_reg_write ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t
		length );
int8_t bmp280_spi_reg_read ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t
		length );
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float zmiana = 0;
int przycisk_nacisniety = 0;


char Received[4];
int wypelnienie=0;
int temperatura_zadana=0;
float pid_error=0.0;
float pid_procent=8.0;
arm_pid_instance_f32 PID;
int SWV_pomiar;
int SWV_pid_error;
int SWV_wyp;
int radiator_state=0;
int size;
char buffer[50];
char flag;
GPIO_PinState LD1_state=GPIO_PIN_RESET;
GPIO_PinState LD3_state=GPIO_PIN_RESET;
GPIO_PinState RAD_State=GPIO_PIN_RESET;
int8_t BMP280_current;
int BMP280_expected = 2200;
struct bmp280_dev bmp280_1 ={
		.dev_id = BMP280_CS1,
		.intf = BMP280_SPI_INTF,
		.read = bmp280_spi_reg_read,
		.write = bmp280_spi_reg_write,
		.delay_ms = HAL_Delay
};
struct bmp280_uncomp_data bmp280_1_data;
int temp32;
uint8_t button_pressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_GPIO_ReadPin(Przycisk_1_GPIO_Port, Przycisk_1_Pin);
	if(GPIO_Pin==Przycisk_1_Pin)
	{
		if(przycisk_nacisniety == 0)
		{
			przycisk_nacisniety = 1;
			zmiana = 1000;
		}
		else if(przycisk_nacisniety == 1)
		{
			przycisk_nacisniety = 2;
			zmiana = 100;
		}
		else if(przycisk_nacisniety == 2)
		{
			przycisk_nacisniety = 0;
			zmiana = 10;
		}

	}

	HAL_GPIO_ReadPin(Przycisk_2_GPIO_Port, Przycisk_2_Pin);
	if(GPIO_Pin==Przycisk_2_Pin)
	{
		BMP280_expected = BMP280_expected + zmiana;
	}

	HAL_GPIO_ReadPin(Przycisk_3_GPIO_Port, Przycisk_3_Pin);
	if(GPIO_Pin==Przycisk_3_Pin)
	{
		BMP280_expected = BMP280_expected - zmiana;
	}
	HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin);
	if(GPIO_Pin == USER_Btn_Pin)
	{
		if(button_pressed == 0)
		{
			button_pressed = 1;
		}
		else if(button_pressed == 1)
		{
			button_pressed = 2;
		}
		else if(button_pressed == 2)
		{
			button_pressed = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART3){
		//HAL_UART_Receive_IT(&huart3, (uint8_t*)Received, 5);
		//HAL_UART_Receive_IT(&huart3, (uint8_t*)kod, 5);
		sscanf(Received, "%d", &BMP280_expected);



	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){

		//obliczenie uchybu
		pid_error =BMP280_expected-temp32;
		//regulacja pwm
		wypelnienie = arm_pid_f32(&PID, pid_error);

		SWV_wyp=wypelnienie;
		//sprawdzenie czy pwm nie wychodzi poza skale ustalona w konfiguracji
		if (wypelnienie > 999){
			wypelnienie = 999;
		} else if (wypelnienie < 0){
			wypelnienie = 0;
		}

		if(BMP280_expected<2200){
			BMP280_expected=2200;
		}
		if(BMP280_expected>3000){
			BMP280_expected=3000;
		}

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, wypelnienie);
		//sprawdzanie czy uchyb jest wiekszy niz 1%
		if(pid_error>pid_procent/2){
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		}
		else {
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}
		if(temp32>abs(BMP280_expected+pid_procent)){
			HAL_GPIO_WritePin(RAD_GPIO_Port, RAD_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(RAD_GPIO_Port, RAD_Pin, GPIO_PIN_RESET);
		}

	}
	if(htim->Instance == TIM4){


		//size = sprintf(buffer, "{\"temperature\":%.2f}\r\n",(float)temp32/100);
		//HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 100);

		size = sprintf(buffer, "Zadana temperatura [*C]: %.2f ; ",(float)BMP280_expected/100);
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 100);
		size = sprintf(buffer, "aktualna temperatura [*C]: %.2f ;",(float)temp32/100);
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 100);
		size = sprintf(buffer, "wypelnienie PWM: %d ; ", wypelnienie);
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 100);
		size = sprintf(buffer, "error [*C]: %.2f ; \r\n", (float)pid_error/100);
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, size, 100);


	}



}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PID.Kp = PID_PARAM_KP;
	PID.Ki = PID_PARAM_KI;
	PID.Kd = PID_PARAM_KD;
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */

	struct bmp280_config conf;
	BMP280_current = bmp280_init(&bmp280_1);
	BMP280_current = bmp280_get_config(&conf, &bmp280_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	arm_pid_init_f32(&PID, 1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start(&htim5);
	LCD_init();

	/* configuring the temperature over sampling, filter coefficient and output data rate */
	/* Overwrite the desired settings */
	conf.filter = BMP280_FILTER_OFF;


	/* Temperature over sampling set at 1x */
	conf.os_temp = BMP280_OS_1X;


	/* Setting the output data rate as 4HZ(250ms) */
	conf.odr = BMP280_ODR_1000_MS;
	BMP280_current = bmp280_set_config(&conf, &bmp280_1);
	BMP280_current = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp280_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		/*Reading the raw data from sensor*/
		BMP280_current = bmp280_get_uncomp_data(&bmp280_1_data, &bmp280_1);
		/*Getting  teh 32 bit compensated temperatue*/
		BMP280_current = bmp280_get_comp_temp_32bit(&temp32, bmp280_1_data.uncomp_temp, &bmp280_1);
		SWV_pid_error=(uint8_t) pid_error;
		SWV_pomiar=temp32;
		RAD_State=HAL_GPIO_ReadPin(RAD_GPIO_Port,RAD_Pin);
		//red diod
		LD3_state= HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin);
		//green diod
		LD1_state= HAL_GPIO_ReadPin(LD1_GPIO_Port, LD1_Pin);
		HAL_UART_Receive_IT(&huart3, (uint8_t*)Received, 4);

		if(button_pressed == 0)
		{
			LCD_goto_line(0);
			LCD_printf("PROJEKT  ");
			LCD_goto_line(1);
			LCD_printf("PAWEL MIKOLAJ ");
		}
		else if(button_pressed == 1)
		{
				LCD_goto_line(0);
				LCD_printf( "Actual=%0.2f[*C]",(float)temp32/100);
				LCD_goto_line(1);
				LCD_printf( "Diff=%.3f[*C]",(float)pid_error/100);
		}
		else if(button_pressed == 2)
		{
			LCD_goto_line(0);
			LCD_printf("Wprowadz temp.  ");
			LCD_goto_line(1);
			LCD_printf( "%0.f Skok: %0.1f    ", (float)BMP280_expected/100, zmiana/100);

		}
		bmp280_1.delay_ms(1000);



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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
}

/* USER CODE BEGIN 4 */
#define BMP280_SPI_BUFFER_LEN  28
#define BMP280_DATA_INDEX 1
int8_t bmp280_spi_reg_write ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t
		length )
{
	/* Implement the SPI write routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ BMP280_SPI_BUFFER_LEN ];
	uint8_t stringpos ;

	/* Copy register address and data to tx buffer */
	txarray [0] = reg_addr ;
	for ( stringpos = 0; stringpos < length ; stringpos ++)
	{
		txarray [ stringpos + BMP280_DATA_INDEX ] = reg_data [ stringpos ];
	}
	// memcpy ( txarray + BMP280_DATA_INDEX , reg_data , length );

	/* Software slave selection procedure */
	if( cs == BMP280_CS1 )
		HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_RESET ) ;

	/* Data exchange */
	status = HAL_SPI_Transmit ( BMP280_SPI , ( uint8_t *) (& txarray ) , length +1 , 100) ;
	while ( BMP280_SPI -> State == HAL_SPI_STATE_BUSY ) {};

	/* Disable all slaves */
	HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_SET ) ;

	if ( status != HAL_OK )
	{
		// The BMP280 API calls for 0 return value as a success , and -1 returned as
		//failure
		iError = ( -1) ;
	}

	return ( int8_t ) iError ;
}

int8_t bmp280_spi_reg_read ( uint8_t cs , uint8_t reg_addr , uint8_t * reg_data , uint16_t
		length )
{
	/* Implement the SPI read routine according to the target machine . */
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ BMP280_SPI_BUFFER_LEN ] = {0 ,};
	uint8_t rxarray [ BMP280_SPI_BUFFER_LEN ] = {0 ,};
	uint8_t stringpos ;

	txarray [0] = reg_addr ;

	/* Software slave selection procedure */
	if( cs == BMP280_CS1 )
		HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_RESET ) ;

	/* Data exchange */
	status = HAL_SPI_TransmitReceive ( BMP280_SPI , ( uint8_t *) (& txarray ) , ( uint8_t *) (&
			rxarray ) , length +1 , 5) ;
	while ( BMP280_SPI -> State == HAL_SPI_STATE_BUSY ) {};

	/* Disable all slaves */
	HAL_GPIO_WritePin ( BMP280_CS1_GPIO_Port , BMP280_CS1_Pin , GPIO_PIN_SET ) ;

	/* Copy data from rx buffer */
	for ( stringpos = 0; stringpos < length ; stringpos ++)
	{
		reg_data [ stringpos ] = rxarray [ stringpos + BMP280_DATA_INDEX ];
	}
	// memcpy ( reg_data , rxarray + BMP280_DATA_INDEX , length );

	if ( status != HAL_OK )
	{
		// The BME280 API calls for 0 return value as a success , and -1 returned as
		//failure
		iError = ( -1) ;
	}

	return ( int8_t ) iError ;
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

