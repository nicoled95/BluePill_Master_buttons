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
  *******************************************************************************
  *El siguiente programa envia datos por medio de protocolo SPI si se preciona un boton
  *Si se acciona un boton 1 conectado en el pin PB12 se envia un 0x01
  *Si se acciona un boton 2 conectado en el pin PB13 se envia un 0x02
  *Ademas cada vez que se envia un valor se enciende el LED sobre la placa
  ******************************************************************************
  *The following program sends data via SPI protocol if a button is pressed
   *If a button 1 connected to pin PB12 is activated, a 0x01 is sent
   *If a button 2 connected to pin PB13 is activated, a 0x02 is sent
   *If In addition, every time a value is sent, the LED on the board lights up.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
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
uint8_t dato_tx[1] ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, LED_pin13_Pin,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

 if( HAL_GPIO_ReadPin(Boton1_GPIO_Port, Boton1_Pin) == 1) //Si el boton es precionado
    {
    	HAL_Delay(5); // funcion antirebote
    	while(HAL_GPIO_ReadPin(Boton1_GPIO_Port, Boton1_Pin)==1);
    	HAL_Delay(5);
		HAL_GPIO_WritePin(GPIOC, LED_pin13_Pin,GPIO_PIN_RESET);

    	dato_tx[0] = 0x01 ; //dato a transmitir para el primer LED del esclavo
    		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //selector de esclavo en esado LOW es seleccionado
    		HAL_SPI_Transmit(&hspi1, dato_tx, 1, HAL_MAX_DELAY); //(SPI utilizado , buffer, tamaño de los datos, tiempo maximo transmision)
    		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); // desabilito el esclavo con una señal de HIGH

    		HAL_Delay(100);
    		HAL_GPIO_WritePin(GPIOC, LED_pin13_Pin,GPIO_PIN_SET);//Prendo un LED
    }
    if( HAL_GPIO_ReadPin(Boton2_GPIO_Port, Boton2_Pin) == 1) //Si el boton es precionado
    {
    	HAL_Delay(5); // funcion antirebote
    	while(HAL_GPIO_ReadPin(Boton2_GPIO_Port, Boton2_Pin)==1);
    	HAL_Delay(5);
    	HAL_GPIO_WritePin(GPIOC, LED_pin13_Pin,GPIO_PIN_RESET);

    	dato_tx[0] = 0x02 ; //dato a transmitir para el segundo LED del exclavo
    	    	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //selector de esclavo en esado LOW es seleccionado
    	    	HAL_SPI_Transmit(&hspi1, dato_tx, 1, HAL_MAX_DELAY); //(SPI utilizado , buffer, tamaño de los datos, tiempo maximo transmision)
    	    	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); // desabilito el esclavo con una señal de HIGH

    	    	HAL_Delay(100);
        		HAL_GPIO_WritePin(GPIOC, LED_pin13_Pin,GPIO_PIN_SET);//Prendo un LED

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
