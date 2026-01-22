/* USER CODE BEGIN Header */
/**
  * STM32F407 - FIX: UART MSP INIT ADDED
  * Amaç: PA2 ve PA3 pinlerini UART moduna zorlamak.
  * Slave ID: 1
  * Baud: 9600
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_hal_flash_ex.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINE_BUFFER_LEN 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char rx_buffer[LINE_BUFFER_LEN];
uint8_t rx_index = 0;
uint8_t slave_id = SLAVE_ID_DEFAULT;  /* Flash'tan okunacak */

/* Motor pozisyonları (0-600mm) */
int16_t motor_positions[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Encoder timer mapping (Motor ID -> Timer Handle) */
TIM_HandleTypeDef* motor_encoders[9] = {
    &htim1,  /* Motor 1 -> TIM1 */
    &htim2,  /* Motor 2 -> TIM2 */
    &htim3,  /* Motor 3 -> TIM3 */
    &htim4,  /* Motor 4 -> TIM4 */
    &htim5,  /* Motor 5 -> TIM5 */
    &htim8,  /* Motor 6 -> TIM8 */
    &htim9,  /* Motor 7 -> TIM9 */
    &htim10, /* Motor 8 -> TIM10 */
    &htim11  /* Motor 9 -> TIM11 */
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
void RS485_TX_Mode(void);
void RS485_RX_Mode(void);
void Send_Response(const char* msg);
void Process_Packet(char* line);
HAL_StatusTypeDef ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len);
uint8_t Flash_Read_SlaveID(void);
HAL_StatusTypeDef Flash_Write_SlaveID(uint8_t id);
void Motor_Control(uint8_t motor_id, int16_t target_position);
void Motor_Stop(uint8_t motor_id);
int32_t Encoder_Read(uint8_t motor_id);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */

  /* 1. FLASH'TAN SLAVE ID OKU */
  slave_id = Flash_Read_SlaveID();
  if (slave_id == 0 || slave_id > 16) {
      /* Geçersiz ID, default değeri kullan (0 = ID atanmamış) */
      slave_id = SLAVE_ID_DEFAULT;
  }
  /* ID okundu, artık slave_id değişkeni kullanılacak */
  
  /* 2. ENCODER TIMER'LARI BAŞLAT */
  /* Encoder mode timer'ları başlat (pozisyon okuma için) */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  
  /* Input Capture timer'ları başlat (TIM9, TIM10, TIM11) */
  HAL_TIM_IC_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim11, TIM_CHANNEL_1);
  
  /* 3. TÜM MOTORLARI DURDUR (Başlangıç güvenliği) */
  for (uint8_t i = 1; i <= 9; i++) {
      Motor_Stop(i);
  }

  /* 4. BOOT TESTİ (5 Kere Hızlı Yanıp Sönme) */
  for(int i=0; i<5; i++) {
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
      HAL_Delay(50);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_Delay(50);
  }

  /* 3. DİNLEME MODU */
  RS485_RX_Mode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Veri Geldi mi? */
    if (ReadLine_NonBlocking(rx_buffer, &rx_index, LINE_BUFFER_LEN) == HAL_OK) {
        Process_Packet(rx_buffer);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim11, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Motor5_F_Pin|Motor5_R_Pin|Motor3_F_Pin|Motor3_R_Pin
                          |Motor6_F_Pin|Motor6_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Motor1_F_Pin|Motor1_R_Pin|GPIO_PIN_0|Motor7_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Moto8_R_Pin|Motor8_F_Pin|Motor2_F_Pin|Motor2_R_Pin
                          |Motor9_F_Pin|Motor9_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Motor4_F_Pin|Motor4_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_DE_RE_GPIO_Port, RS485_DE_RE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Motor5_F_Pin Motor5_R_Pin Motor3_F_Pin Motor3_R_Pin
                           Motor6_F_Pin Motor6_R_Pin */
  GPIO_InitStruct.Pin = Motor5_F_Pin|Motor5_R_Pin|Motor3_F_Pin|Motor3_R_Pin
                          |Motor6_F_Pin|Motor6_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1_F_Pin Motor1_R_Pin PE0 Motor7_R_Pin */
  GPIO_InitStruct.Pin = Motor1_F_Pin|Motor1_R_Pin|GPIO_PIN_0|Motor7_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* LED (PE4) - Boot test ve haberleşme göstergesi için */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Moto8_R_Pin Motor8_F_Pin Motor2_F_Pin Motor2_R_Pin
                           Motor9_F_Pin Motor9_R_Pin */
  GPIO_InitStruct.Pin = Moto8_R_Pin|Motor8_F_Pin|Motor2_F_Pin|Motor2_R_Pin
                          |Motor9_F_Pin|Motor9_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor4_F_Pin Motor4_R_Pin */
  GPIO_InitStruct.Pin = Motor4_F_Pin|Motor4_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DE_RE_Pin */
  GPIO_InitStruct.Pin = RS485_DE_RE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_DE_RE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* --- MOTOR KONTROL FONKSİYONLARI --- */

/**
  * @brief  Motor pinlerini döndürür (Motor ID -> GPIO Pin)
  */
static void Get_Motor_Pins(uint8_t motor_id, GPIO_TypeDef** port_f, uint16_t* pin_f, GPIO_TypeDef** port_r, uint16_t* pin_r) {
    switch(motor_id) {
        case 1: *port_f = Motor1_F_GPIO_Port; *pin_f = Motor1_F_Pin; *port_r = Motor1_R_GPIO_Port; *pin_r = Motor1_R_Pin; break;
        case 2: *port_f = Motor2_F_GPIO_Port; *pin_f = Motor2_F_Pin; *port_r = Motor2_R_GPIO_Port; *pin_r = Motor2_R_Pin; break;
        case 3: *port_f = Motor3_F_GPIO_Port; *pin_f = Motor3_F_Pin; *port_r = Motor3_R_GPIO_Port; *pin_r = Motor3_R_Pin; break;
        case 4: *port_f = Motor4_F_GPIO_Port; *pin_f = Motor4_F_Pin; *port_r = Motor4_R_GPIO_Port; *pin_r = Motor4_R_Pin; break;
        case 5: *port_f = Motor5_F_GPIO_Port; *pin_f = Motor5_F_Pin; *port_r = Motor5_R_GPIO_Port; *pin_r = Motor5_R_Pin; break;
        case 6: *port_f = Motor6_F_GPIO_Port; *pin_f = Motor6_F_Pin; *port_r = Motor6_R_GPIO_Port; *pin_r = Motor6_R_Pin; break;
        case 7: *port_f = GPIOE; *pin_f = GPIO_PIN_0; *port_r = Motor7_R_GPIO_Port; *pin_r = Motor7_R_Pin; break; // Motor7 Forward: PE0, Reverse: PE1
        case 8: *port_f = Motor8_F_GPIO_Port; *pin_f = Motor8_F_Pin; *port_r = Moto8_R_GPIO_Port; *pin_r = Moto8_R_Pin; break;
        case 9: *port_f = Motor9_F_GPIO_Port; *pin_f = Motor9_F_Pin; *port_r = Motor9_R_GPIO_Port; *pin_r = Motor9_R_Pin; break;
        default: *port_f = NULL; *pin_f = 0; *port_r = NULL; *pin_r = 0; break;
    }
}

/**
  * @brief  Motoru durdur
  */
void Motor_Stop(uint8_t motor_id) {
    if (motor_id < 1 || motor_id > 9) return;
    
    GPIO_TypeDef *port_f, *port_r;
    uint16_t pin_f, pin_r;
    Get_Motor_Pins(motor_id, &port_f, &pin_f, &port_r, &pin_r);
    
    /* Forward ve Reverse pinlerini LOW yap (motor durur) */
    if (port_f && pin_f) HAL_GPIO_WritePin(port_f, pin_f, GPIO_PIN_RESET);
    if (port_r && pin_r) HAL_GPIO_WritePin(port_r, pin_r, GPIO_PIN_RESET);
}

/**
  * @brief  Encoder değerini okur (Encoder Mode veya Timer Mode fark etmez)
  */
int32_t Encoder_Read(uint8_t motor_id) {
    if (motor_id < 1 || motor_id > 9) return 0;
    
    TIM_HandleTypeDef* htim = motor_encoders[motor_id - 1];
    
    /* Timer başlatılmış mı kontrol et */
    if (htim->Instance == NULL) return 0;
    
    /* Encoder mode için counter oku */
    if (htim->Instance == TIM1 || htim->Instance == TIM2 || htim->Instance == TIM3 || 
        htim->Instance == TIM4 || htim->Instance == TIM5 || htim->Instance == TIM8) {
        /* Encoder Mode: Counter değerini oku */
        return (int32_t)__HAL_TIM_GET_COUNTER(htim);
    }
    /* Input Capture Mode için (TIM9, TIM10, TIM11) - şimdilik 0 döndür */
    return 0;
}

/**
  * @brief  Motoru hedef pozisyona sürer (Encoder ile pozisyon kontrolü)
  */
void Motor_Control(uint8_t motor_id, int16_t target_position) {
    if (motor_id < 1 || motor_id > 9) return;
    if (target_position < 0 || target_position > 600) return;
    
    GPIO_TypeDef *port_f, *port_r;
    uint16_t pin_f, pin_r;
    Get_Motor_Pins(motor_id, &port_f, &pin_f, &port_r, &pin_r);
    
    /* Mevcut encoder değerini oku (Encoder Mode veya Timer Mode fark etmez) */
    int32_t current_encoder = Encoder_Read(motor_id);
    
    /* Hedef pozisyonu kaydet */
    motor_positions[motor_id - 1] = target_position;
    
    /* NOT: Encoder değerini mm'ye çevirmek için kalibrasyon gerekir */
    /* Encoder kalibrasyonu: pulse/mm oranı belirlenmeli (örn: 200 pulse/mm) */
    /* Şimdilik basit kontrol: Encoder değerine göre motor yönü belirlenir */
    
    /* Motoru durdur */
    Motor_Stop(motor_id);
    
    /* Basit motor kontrolü - Encoder değerine göre yön belirleme */
    /* NOT: Bu kısım encoder kalibrasyonu yapıldıktan sonra güncellenecek */
    /* Örnek kalibrasyon: 200 pulse = 1mm, 600mm = 120,000 pulse */
    
    /* Şimdilik: Hedef pozisyon > mevcut encoder/pulse_ratio ise Forward */
    /* Encoder kalibrasyonu yapıldıktan sonra bu kısım geliştirilecek */
    /* İleride: PID kontrolü veya daha gelişmiş pozisyon kontrolü eklenecek */
}

/* --- RS485 KONTROL FONKSİYONLARI --- */

void RS485_TX_Mode(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void RS485_RX_Mode(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void Send_Response(const char* msg) {
    HAL_Delay(20); // Master dinlemeye geçsin (Flash yazma sonrası daha fazla bekle)
    RS485_TX_Mode();
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
    while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET); // Tüm veri gönderilene kadar bekle
    HAL_Delay(5); // Son bit'in gittiğinden emin ol
    RS485_RX_Mode();
}

HAL_StatusTypeDef ReadLine_NonBlocking(char *dst, uint8_t *pos, size_t max_len) {
    uint8_t ch;
    // Timeout 0 -> Bekleme yapma, veri yoksa devam et
    if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK) {
        if (ch == '\n') {
            dst[*pos] = '\0';
            *pos = 0;
            return HAL_OK;
        } else if (ch != '\r') {
            if (*pos < max_len - 1) dst[(*pos)++] = (char)ch;
        }
    }
    return HAL_BUSY;
}

void Process_Packet(char* line) {
    char tempLine[LINE_BUFFER_LEN];
    strncpy(tempLine, line, LINE_BUFFER_LEN);

    char *cmd = strtok(tempLine, ":");
    char *param1 = strtok(NULL, ":");
    char *param2 = strtok(NULL, ":");
    
    if (cmd == NULL) return;

    /* SETID komutu: SETID:ID (Broadcast, ID kontrolü yok) */
    if (strcmp(cmd, "SETID") == 0) {
        if (param1 != NULL) {
            int newID = atoi(param1);
            if (newID >= 1 && newID <= 16) {
                HAL_StatusTypeDef flash_status = Flash_Write_SlaveID((uint8_t)newID);
                if (flash_status == HAL_OK) {
                    /* Flash yazma başarılı, ID'yi güncelle */
                    slave_id = (uint8_t)newID;
                    
                    /* Flash'tan tekrar oku ve doğrula */
                    uint8_t verify_id = Flash_Read_SlaveID();
                    if (verify_id == slave_id) {
                        /* Doğrulama başarılı, yanıt gönder */
                        char response[32];
                        snprintf(response, sizeof(response), "IDSET:%02d\n", slave_id);
                        Send_Response(response);
                        
                        /* Başarılı ID ayarı için LED'i 3 kez yanıp söndür */
                        for(int i=0; i<3; i++) {
                            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
                            HAL_Delay(100);
                            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
                            HAL_Delay(100);
                        }
                    } else {
                        /* Doğrulama başarısız */
                        Send_Response("IDERR:Verification failed\n");
                    }
                } else {
                    /* Flash yazma başarısız */
                    Send_Response("IDERR:Flash write failed\n");
                }
            } else {
                Send_Response("IDERR:Invalid ID (1-16)\n");
            }
        }
        return;
    }

    /* Diğer komutlar için target ID kontrolü */
    if (param1 == NULL) return;
    
    int targetID = atoi(param1);

    if (targetID == slave_id) {
        /* MESAJ GELDİ! LED'İ UZUN YAK */
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
        
        if (strcmp(cmd, "PING") == 0) {
            /* PONG göndermeden önce 4 saniye bekle */
            HAL_Delay(4000);
            char response[32];
            snprintf(response, sizeof(response), "PONG:%02d\n", slave_id);
            Send_Response(response);
        }
        else if (strcmp(cmd, "MOV") == 0) {
            /* MOV:SlaveID:MotorID:Value */
            if (param2 != NULL) {
                int motorID = atoi(param2);
                int targetValue = atoi(strtok(NULL, ":"));
                
                if (motorID >= 1 && motorID <= 9 && targetValue >= 0 && targetValue <= 600) {
                    Motor_Control((uint8_t)motorID, (int16_t)targetValue);
                    char response[32];
                    snprintf(response, sizeof(response), "MOVOK:%02d:%02d:%d\n", slave_id, motorID, targetValue);
                    Send_Response(response);
                } else {
                    Send_Response("MOVERR:Invalid params\n");
                }
            }
        }
        else if (strcmp(cmd, "ALL") == 0) {
            /* ALL:SlaveID:Value */
            int targetValue = atoi(param2);
            
            if (targetValue >= 0 && targetValue <= 600) {
                /* Tüm motorları aynı pozisyona sür */
                for (uint8_t m = 1; m <= 9; m++) {
                    Motor_Control(m, (int16_t)targetValue);
                }
                char response[32];
                snprintf(response, sizeof(response), "ALLOK:%02d:%d\n", slave_id, targetValue);
                Send_Response(response);
            } else {
                Send_Response("ALLERR:Invalid value (0-600)\n");
            }
        }
        
        /* GÖRÜLMESİ İÇİN BEKLE VE SÖNDÜR */
        HAL_Delay(100); 
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
    }
}

/* --- FLASH MEMORY FONKSİYONLARI (EEPROM Emülasyonu) --- */

/**
  * @brief  Flash'tan Slave ID okur
  * @retval Slave ID (1-16) veya 0 (geçersiz/boş)
  */
uint8_t Flash_Read_SlaveID(void) {
    /* Flash'tan 32-bit word oku */
    uint32_t flash_data = *((volatile uint32_t*)SLAVE_ID_FLASH_ADDRESS);
    
    /* Flash boş mu kontrol et (0xFFFFFFFF = boş) */
    if (flash_data == 0xFFFFFFFF || flash_data == 0x00000000) {
        return SLAVE_ID_DEFAULT;
    }
    
    /* Magic number kontrolü */
    uint32_t magic = (flash_data >> 16) & 0xFFFF;
    uint32_t id = flash_data & 0xFFFF;
    
    /* Magic number ve ID geçerliliği kontrolü */
    if (magic == (SLAVE_ID_MAGIC & 0xFFFF) && id >= 1 && id <= 16) {
        return (uint8_t)id;
    }
    
    /* Geçersiz veri */
    return SLAVE_ID_DEFAULT;
}

/**
  * @brief  Slave ID'yi Flash'a yazar
  * @param  id: Yazılacak ID (1-16)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Flash_Write_SlaveID(uint8_t id) {
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;
    
    /* ID kontrolü */
    if (id < 1 || id > 16) {
        return HAL_ERROR;
    }
    
    /* Flash Unlock */
    if (HAL_FLASH_Unlock() != HAL_OK) {
        return HAL_ERROR;
    }
    
    /* Flash'taki mevcut değeri kontrol et */
    uint32_t current_data = *((volatile uint32_t*)SLAVE_ID_FLASH_ADDRESS);
    uint32_t new_data = ((SLAVE_ID_MAGIC & 0xFFFF) << 16) | id;
    
    /* Eğer aynı değer yazılmak isteniyorsa, erase etmeye gerek yok */
    if (current_data == new_data) {
        HAL_FLASH_Lock();
        return HAL_OK;
    }
    
    /* Son sector'ü erase et (STM32F407VETX: Sector 7, 128KB) */
    /* Flash: 512KB (0x08000000 - 0x0807FFFF) */
    /* Sector yapısı: */
    /*   Sector 0-3: 16KB each (0x08000000 - 0x0800FFFF) */
    /*   Sector 4: 64KB (0x08010000 - 0x0801FFFF) */
    /*   Sector 5: 128KB (0x08020000 - 0x0803FFFF) */
    /*   Sector 6: 128KB (0x08040000 - 0x0805FFFF) */
    /*   Sector 7: 128KB (0x08060000 - 0x0807FFFF) <- SON SECTOR! */
    /* Son 4 byte: 0x0807FFFC, bu Sector 7'de */
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_7;  /* Son sector (512KB Flash için) */
    EraseInitStruct.NbSectors = 1;
    
    status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }
    
    /* Yeni değeri yaz */
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SLAVE_ID_FLASH_ADDRESS, new_data);
    
    /* Flash Lock */
    HAL_FLASH_Lock();
    
    return status;
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
#ifdef USE_FULL_ASSERT
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
