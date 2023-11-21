/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DHT.h"
#include "delay_timer.h"
#include "LoRa.h"
#include "i2c-lcd.h"
#include <stdio.h>
#include "EEPROM.h"
#include "BH1750.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ID_Node 1
#define READ 	0
#define	WRITE	1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
DHT_Name DHT1;

LoRa myLoRa;
uint8_t LoRa_stat = 0;

uint16_t HumiSoil;
uint32_t buffer;

uint8_t EEPROM_Data[10];

uint8_t receive_data[128];
uint8_t transmit_data[128];
uint8_t packet_size = 0;
uint8_t flgLoRa = 0;


uint16_t Lux;
uint8_t Lux_H;
uint8_t Lux_L;
uint8_t LuxUpper_H = 0;
uint8_t LuxUpper_L = 150;
uint8_t LuxLower_H = 0;
uint8_t LuxLower_L = 70;

uint8_t Auto=0;
uint8_t Bump=0;
uint8_t Fan=0;
uint8_t Led=0;
uint8_t TempUpper=30;
uint8_t	TempLower=20;
uint8_t HumiUpper=80;
uint8_t HumiLower=50;
uint16_t LuxUpper=150;
uint16_t LuxLower=75;
uint8_t HumiSoilUpper=60;
uint8_t HumiSoilLower=70;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t map_lcd(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax);
void LoRa_ACK();
void Read_Eeprom();
void Write_Eeprom();
void Control();
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char str[10];
	uint32_t Lcd_Tick;
	uint32_t Ctrl_Tick;
	
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();			//create LCD
	lcd_clear();		//clear LCD

	//create BH1750
	BH1750_device_t* test_dev = BH1750_init_dev_struct(&hi2c1, "test device", true);
  BH1750_init_dev(test_dev);

	DHT_Init(&DHT1, DHT11, &htim4, DHT11_GPIO_Port, DHT11_Pin);		//create DHT
	HAL_ADC_Start_DMA(&hadc1,&buffer,1);	//Start read TempSoil
	
//	Write_Eeprom();

	Read_Eeprom();
	
	myLoRa = newLoRa();
	
	myLoRa.CS_port         = NSS_GPIO_Port;
	myLoRa.CS_pin          = NSS_Pin;
	myLoRa.reset_port      = RST_GPIO_Port;
	myLoRa.reset_pin       = RST_Pin;
	myLoRa.DIO0_port       = DIO0_GPIO_Port;
	myLoRa.DIO0_pin        = DIO0_Pin;
	myLoRa.hSPIx           = &hspi1;
	
	if(LoRa_init(&myLoRa)==LORA_OK){
		LoRa_stat = 1;
		lcd_send_cmd(0x80|0x00);
		lcd_send_string("Lora OK");
		HAL_Delay(1000);
		lcd_send_cmd(0x80|0x00);
		lcd_send_string("       ");
	}

	//Receive from Gateway
	LoRa_startReceiving(&myLoRa);

	Lcd_Tick = HAL_GetTick() - 2000;
	Ctrl_Tick = HAL_GetTick() - 500;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//LoRa receive
		if(flgLoRa == 1)
		{
			flgLoRa = 0;
			packet_size = LoRa_receive(&myLoRa, receive_data, 128);
			//Receive
			if(receive_data[0]== ID_Node)	//ID_Node
			{
				if(receive_data[1] == READ)
				{	Lux_H = Lux >> 8;
					Lux_L = Lux & 0xff;
					transmit_data[0] = ID_Node;
					transmit_data[1] = Auto;
					transmit_data[2] = Bump;
					transmit_data[3] = Fan;
					transmit_data[4] = Led;
					transmit_data[5] = DHT1.Temp;
					transmit_data[6] = DHT1.Humi;
					transmit_data[7] = Lux_H;
					transmit_data[8] = Lux_L;
					transmit_data[9] = HumiSoil;
					LoRa_transmit(&myLoRa, transmit_data, 10, 500);
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				}
				else if(receive_data[1] == WRITE)
				{
					LoRa_ACK();
					Auto = receive_data[2];
					Bump = receive_data[3];
					Fan = receive_data[4];
					Led = receive_data[5];
					Control();
					//check update eeprom
					if((TempUpper != receive_data[6])||(TempLower != receive_data[7])||(HumiUpper != receive_data[8])||(HumiLower != receive_data[9])||(LuxUpper_H != receive_data[10])
						||(LuxUpper_L != receive_data[11])||(LuxLower_H != receive_data[12])||(LuxLower_L != receive_data[13])||(HumiSoilUpper != receive_data[14])||(HumiSoilLower != receive_data[15]))
					{ 
						if ((receive_data[6] != 0)||(receive_data[7] != 0)||(receive_data[8] != 0)||(receive_data[9] != 0)||(receive_data[10] != 0)
						||(receive_data[11] != 0)||(receive_data[12] != 0)||(receive_data[13] != 0)||(receive_data[14] != 0)||(receive_data[15] != 0))
						{
							TempUpper = receive_data[6];
							TempLower = receive_data[7];
							HumiUpper = receive_data[8];
							HumiLower = receive_data[9];
							LuxUpper_H = receive_data[10];
							LuxUpper_L = receive_data[11];
							LuxLower_H = receive_data[12];
							LuxLower_L = receive_data[13];
							HumiSoilUpper = receive_data[14];
							HumiSoilLower = receive_data[15];
							Write_Eeprom();
						}
					}
				}
			}	
		}
		
		
		//Lcd
		if(HAL_GetTick() - Lcd_Tick >= 2000)
		{
			//Get tick
			Lcd_Tick = HAL_GetTick(); 
			
			lcd_send_cmd(0x80|0x00);
			lcd_send_string("TEMP: ");
			lcd_send_cmd(0x80|0x40);
			lcd_send_string("HUMI: ");
			lcd_send_cmd(0x80|0x0A);
			lcd_send_string("HSoil: ");
			lcd_send_cmd(0x80|0x4A);
			lcd_send_string("L: ");
			
			lcd_send_cmd(0x80|0x14);
			lcd_send_string("AUTO ");
			lcd_send_cmd(0x80|0x19);
			lcd_send_string("BUMP ");
			lcd_send_cmd(0x80|0x1E);
			lcd_send_string(" FAN ");
			lcd_send_cmd(0x80|0x23);
			lcd_send_string(" LED ");
			
			//DHT11
			if(!DHT_ReadTempHum(&DHT1))
			{	
				lcd_send_cmd(0x80|0x06);
				sprintf(str, "%dC", DHT1.Temp);
				lcd_send_string(str);
				
				lcd_send_cmd(0x80|0x46);
				sprintf(str, "%d%%", DHT1.Humi);
				lcd_send_string(str);				
			}
			
			//HumiSoil
			HumiSoil = 100 - map_lcd(buffer,0,4095,0,100);
//			HumiSoil = (uint16_t)(100 - 3.3*(buffer/(float)4095));
			lcd_send_cmd(0x80|0x11);
			sprintf(str, "%d%%", HumiSoil);
			lcd_send_string(str);
			
			//BH1750
			lcd_send_cmd(0x80|0x4D);
			lcd_send_string("       ");
			lcd_send_cmd(0x80|0x4C);
			sprintf(str, "%dLux", Lux);
			lcd_send_string(str);
										
		}
		
		//Controll in Node
		if(HAL_GetTick() - Ctrl_Tick >= 300)
		{
			//Get tick
			Ctrl_Tick = HAL_GetTick();
			
			//Update BH1750
			test_dev->poll(test_dev);
			Lux = test_dev->value;
			
			//Compare
			if(Auto == 1)
			{
				//Controll TempAir
				if(DHT1.Temp < TempLower)
				{
					Fan = 1;
					HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, GPIO_PIN_SET);
				}
				else if(DHT1.Temp > TempUpper)
				{
					Fan = 0;
					HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, GPIO_PIN_RESET);
				}
				
				//Controll HumiSoil
				HumiSoil = 100 - map_lcd(buffer, 0, 4095, 0, 100);
				if(HumiSoil < HumiSoilLower)
				{
					Bump = 1;
					HAL_GPIO_WritePin(BUMP_OUT_GPIO_Port, BUMP_OUT_Pin, GPIO_PIN_SET);
				}
				else if(HumiSoil > HumiSoilUpper)
				{
					Bump = 0;
					HAL_GPIO_WritePin(BUMP_OUT_GPIO_Port, BUMP_OUT_Pin, GPIO_PIN_RESET);
				}
				
				if(Lux < LuxLower)
				{
					Led = 1;
					HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
				}
				else if(Lux < LuxUpper)
				{
					Led = 0;
					HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_RESET);
				}
				
			}
			
			//Show status controll
			if(Auto == 0)
			{
				lcd_send_cmd(0x80|0x54);
				lcd_send_string(" OFF");
			}
			else
			{
				lcd_send_cmd(0x80|0x54);
				lcd_send_string(" ON ");
			}
			
			if(Bump == 0)
			{
				lcd_send_cmd(0x80|0x59);
				lcd_send_string(" OFF");
			}
			else
			{
				lcd_send_cmd(0x80|0x59);
				lcd_send_string(" ON ");
			}
			
			if(Fan == 0)
			{
				lcd_send_cmd(0x80|0x5E);
				lcd_send_string(" OFF");
			}
			else
			{
				lcd_send_cmd(0x80|0x5E);
				lcd_send_string(" ON ");
			}
			
			if(Led == 0)
			{
				lcd_send_cmd(0x80|0x63);
				lcd_send_string(" OFF");
			}
			else
			{
				lcd_send_cmd(0x80|0x63);
				lcd_send_string(" ON ");
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUMP_OUT_Pin|FAN_OUT_Pin|LED_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_IN_Pin FAN_IN_Pin BUMP_IN_Pin AUTO_Pin */
  GPIO_InitStruct.Pin = LED_IN_Pin|FAN_IN_Pin|BUMP_IN_Pin|AUTO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin RST_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUMP_OUT_Pin FAN_OUT_Pin LED_OUT_Pin */
  GPIO_InitStruct.Pin = BUMP_OUT_Pin|FAN_OUT_Pin|LED_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
uint32_t map_lcd(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
	return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

//Control
void Control()
{
	if (Auto ==0)
	{
		
			HAL_GPIO_WritePin(BUMP_OUT_GPIO_Port, BUMP_OUT_Pin, Bump);
		
			HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, Fan);
		
			HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, Led);

	} 
}

//Interrupt Lora receive and button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t Auto_temp;
	uint8_t Bump_temp;
	uint8_t Fan_temp;
	uint8_t Led_temp;
	
	if(GPIO_Pin == DIO0_Pin) 
	{
		flgLoRa = 1;
		
		HAL_GPIO_WritePin(BUMP_OUT_GPIO_Port, BUMP_OUT_Pin, Bump);
		Bump_temp = HAL_GPIO_ReadPin(BUMP_IN_GPIO_Port, BUMP_IN_Pin);
		Fan_temp = HAL_GPIO_ReadPin(FAN_IN_GPIO_Port, FAN_IN_Pin);
		Led_temp = HAL_GPIO_ReadPin(LED_IN_GPIO_Port, LED_IN_Pin);
	}
	else if(GPIO_Pin == AUTO_Pin)
	{
		Auto_temp = HAL_GPIO_ReadPin(AUTO_GPIO_Port, AUTO_Pin);
		if(Auto_temp == GPIO_PIN_SET)
		{
			Auto = 0;
			
			Bump_temp = HAL_GPIO_ReadPin(BUMP_IN_GPIO_Port, BUMP_IN_Pin);
			if(Bump_temp == GPIO_PIN_RESET)
			{
				Bump = 1;
				HAL_GPIO_WritePin(BUMP_OUT_GPIO_Port, BUMP_OUT_Pin, GPIO_PIN_SET);
			}
			else
			{
				Bump = 0;
				HAL_GPIO_WritePin(BUMP_OUT_GPIO_Port, BUMP_OUT_Pin, GPIO_PIN_RESET);
			}
						
			Fan_temp = HAL_GPIO_ReadPin(FAN_IN_GPIO_Port, FAN_IN_Pin);
			if(Fan_temp == GPIO_PIN_RESET)
			{
				Fan = 1;
				HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, GPIO_PIN_SET);
			}
			else
			{
				Fan = 0;
				HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, GPIO_PIN_RESET);
			}
			
			Led_temp = HAL_GPIO_ReadPin(LED_IN_GPIO_Port, LED_IN_Pin);
			if(Led_temp == GPIO_PIN_RESET)
			{
				Led = 1;
				HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
			}
			else
			{
				Led = 0;
				HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_RESET);
			}			
		}
		else 
		{
			Auto = 1;
		}
	}
	else
	{
		if(Auto == 0)
		{
			if(GPIO_Pin == BUMP_IN_Pin)
			{
				Bump_temp = HAL_GPIO_ReadPin(BUMP_IN_GPIO_Port, BUMP_IN_Pin);
				if(Bump_temp == GPIO_PIN_RESET)
				{
					Bump = 1;
					HAL_GPIO_WritePin(BUMP_OUT_GPIO_Port, BUMP_OUT_Pin, GPIO_PIN_SET);
				}
				else
				{
					Bump = 0;
					HAL_GPIO_WritePin(BUMP_OUT_GPIO_Port, BUMP_OUT_Pin, GPIO_PIN_RESET);
				}
			}
			else if(GPIO_Pin == FAN_IN_Pin)
			{
				Fan_temp = HAL_GPIO_ReadPin(FAN_IN_GPIO_Port, FAN_IN_Pin);
				if(Fan_temp == GPIO_PIN_RESET)
				{
					Fan = 1;
					HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, GPIO_PIN_SET);
				}
				else
				{
					Fan = 0;
					HAL_GPIO_WritePin(FAN_OUT_GPIO_Port, FAN_OUT_Pin, GPIO_PIN_RESET);
				}
			}
			else if(GPIO_Pin == LED_IN_Pin)
			{
				Led_temp = HAL_GPIO_ReadPin(LED_IN_GPIO_Port, LED_IN_Pin);
				if(Led_temp == GPIO_PIN_RESET)
				{
					Led = 1;
					HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_SET);
				}
				else
				{
					Led = 0;
					HAL_GPIO_WritePin(LED_OUT_GPIO_Port, LED_OUT_Pin, GPIO_PIN_RESET);
				}
			}
		}
	}
}

void LoRa_ACK()
{
	transmit_data[0] = ID_Node;
	transmit_data[1] = 255;
	LoRa_transmit(&myLoRa, transmit_data, 2, 300);
}

void Read_Eeprom()
{
	//Read data threshold from eeprom
	EEPROM_Read(4,0,EEPROM_Data,10);
	
	TempUpper = EEPROM_Data[0];
	TempLower = EEPROM_Data[1];
	HumiUpper = EEPROM_Data[2];
	HumiLower = EEPROM_Data[3];
	LuxUpper_H = EEPROM_Data[4];
	LuxUpper_L = EEPROM_Data[5];
	LuxLower_H = EEPROM_Data[6];
	LuxLower_L = EEPROM_Data[7];
	HumiSoilUpper = EEPROM_Data[8];
	HumiSoilLower = EEPROM_Data[9];
	LuxUpper = (((uint16_t)LuxUpper_H) << 8) | ((uint16_t)LuxUpper_L);
	LuxLower = (((uint16_t)LuxLower_H) << 8) | ((uint16_t)LuxLower_L);
	
}

void Write_Eeprom()
{
	LuxUpper = (((uint16_t)LuxUpper_H) << 8) | ((uint16_t)LuxUpper_L);
	LuxLower = (((uint16_t)LuxLower_H) << 8) | ((uint16_t)LuxLower_L);
	//Read data threshold from eeprom
	EEPROM_Data[0] = TempUpper;
	EEPROM_Data[1] = TempLower;
	EEPROM_Data[2] = HumiUpper;
	EEPROM_Data[3] = HumiLower;
	EEPROM_Data[4] = LuxUpper_H;
	EEPROM_Data[5] = LuxUpper_L;
	EEPROM_Data[6] = LuxLower_H;
	EEPROM_Data[7] = LuxLower_L;
	EEPROM_Data[8] = HumiSoilUpper;
	EEPROM_Data[9] = HumiSoilLower;
	
	EEPROM_Write(4,0,EEPROM_Data,10);
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
