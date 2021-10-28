/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "module_uart.h"
#include "mpu9250.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __MPU9250{
    I2C_HandleTypeDef i2c;
    uint8_t gyro_address;
    uint8_t meg_address;
}MPU9250;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_ADDRESS (0x68 << 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_data;
uart_t uart;
bool send_flag = false;

void gyro_who_am_i() //gyro action check
{
	uint8_t i2c_data[2] = {0x75, 0x00};
	uint8_t recvData[1] = {0,};

	uint8_t uartData[5] = {0,};

	HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, &i2c_data[0], 1, 50);
	HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDRESS, recvData, 1, 50);
	sprintf((char *)uartData, "0x%x\n", recvData[0]);
	HAL_UART_Transmit(&huart1, uartData, 5, 50);
}

//use gyro
float f_gyx, f_gyy,f_gyz;
int32_t getmpuaccx,getmpuaccy,getmpuaccz;

void Gyro_Writebyte(MPU9250 * I2C,uint8_t register_address,uint8_t data){
    uint8_t Trans[2]={register_address, data};
    HAL_I2C_Master_Transmit(&(I2C->i2c), I2C->gyro_address,Trans,2,10);
}

uint8_t Gyro_Readbyte(MPU9250 * I2C,uint8_t register_address){
    uint8_t Trans[1]={register_address};
    uint8_t Receive[1];
    HAL_I2C_Master_Transmit(&(I2C->i2c), I2C->gyro_address,Trans,1,10);
    HAL_I2C_Master_Receive(&(I2C->i2c),I2C->gyro_address,Receive,1,10);

    return Receive[0];
}

void init_MPU9250(MPU9250* mpu9250){
    Gyro_Writebyte(mpu9250,PWR_MGMT_1,0x00);

    HAL_Delay(100);
    Gyro_Writebyte(mpu9250,PWR_MGMT_1,0x01);
    Gyro_Writebyte(mpu9250,SMPLRT_DIV,0X07);

    Gyro_Writebyte(mpu9250,GYRO_CONFIG,0x08);
    Gyro_Writebyte(mpu9250,INT_PIN_CFG,0X02);
    Gyro_Writebyte(mpu9250,ACCEL_CONFIG,0x08);
}

void MPU_read_acc_gyro(MPU9250* mpu9250){
    uint8_t databuf[14];
    int16_t ax,ay,az;
    int16_t gx,gy,gz;

    float Axyz[30] = {0,};
    float Gxyz[30] = {0,};

    HAL_I2C_Mem_Read(&(mpu9250->i2c),mpu9250->gyro_address,0x3b,I2C_MEMADD_SIZE_8BIT,databuf,14,10);
    ax=(((int16_t)databuf[0]<<8)|databuf[1]); Axyz[0] = (double) ax / 16384; //accelerometer X-axis value
    ay=(((int16_t)databuf[2]<<8)|databuf[3]); Axyz[1] = (double) ay / 16384; //accelerometer Y-axis value
    az=(((int16_t)databuf[4]<<8)|databuf[5]); Axyz[2] = (double) az / 16384; //accelerometer Z-axis value

    gx=(((int16_t)databuf[8]<<8)|databuf[9]); Gxyz[0] = (double) gx * 250 / 32768; //gyroscope X-axis value
    gy=(((int16_t)databuf[10]<<8)|databuf[11]); Gxyz[1] = (double) gy * 250 / 32768; //gyroscope Y-axis value
    gz=(((int16_t)databuf[12]<<8)|databuf[13]); Gxyz[2] = (double) gz * 250 / 32768; //gyroscope Z-axis value

    sprintf((char *)Axyz, "Accel_x: %.1f, y: %.1f, z: %.1f\n", Axyz[0], Axyz[1], Axyz[2]);
    HAL_UART_Transmit(&huart1, Axyz, strlen((char*)Axyz), 10);

    sprintf((char *)Gxyz, "Giro_x: %.1f, y: %.1f, z: %.1f\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    HAL_UART_Transmit(&huart1, Gxyz, strlen((char*)Gxyz), 10);
}

void MPU_read_mag(MPU9250* mpu9250){
	uint8_t databuf[6];
	int16_t mx,my,mz;
	float Mxyz[30] = {0,};

	HAL_I2C_Master_Transmit(AK8963_ADDRESS, AK8963_CNTL, 0x01, 1, 10); //enable the magnetometer
	HAL_Delay(10);

	HAL_I2C_Master_Receive(AK8963_ADDRESS, AK8963_XOUT_L, databuf, 6, 10);

	mx = (((int16_t)databuf[1]) << 8) | databuf[0];
	my = (((int16_t)databuf[3]) << 8) | databuf[2];
	mz = (((int16_t)databuf[5]) << 8) | databuf[4];

	Mxyz[0] = (double) mx * 1200 / 4096;
	Mxyz[1] = (double) my * 1200 / 4096;
	Mxyz[2] = (double) mz * 1200 / 4096;

	sprintf((char *)Mxyz, "Mag_x: %.1f, y: %.1f, z: %.1f\n", Mxyz[0], Mxyz[1], Mxyz[2]);
	HAL_UART_Transmit(&huart1, Mxyz, strlen((char*)Mxyz), 10);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t data;
	MPU9250 mpu9250={hi2c1,MPU9250_ADDRESS,AK8963_ADDRESS};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  init_uart(&uart);
  init_MPU9250(&mpu9250);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (isEmpty(&uart) == 0) {
		  data = pop(&uart);
		  if(data == '1')
			  gyro_who_am_i();
	  }

	  if(send_flag == true) //1초마다 업데이트
	  {
		  MPU_read_acc_gyro(&mpu9250);
		  MPU_read_mag(&mpu9250);
		  send_flag = false;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7200 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM6) //1Hz
  {
	  send_flag = true;
  }
}

/*Interrupts RX Callback************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    push(&uart, rx_data);
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
