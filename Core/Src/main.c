/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : DATN AMR ROBOT - MPU6050 Direct I2C & Encoder Base
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU6050_normal.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ARR_MAX_L     20999
#define PID_LIMIT_L   ARR_MAX_L

#define ARR_MAX_R     20999
#define PID_LIMIT_R   ARR_MAX_R

#define Ts            0.01f
#define ARR_MIN       10
#define Base_Link     0.31548f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/*=========================== FRAME PARAM ==============================*/
const float WHEEL_RADIUS = 0.05f;
const float DEG_TO_RAD   = 0.0174532925f;

/*=========================== ENCODER PARAM ============================*/
Encoder_t LeftEncoder;
Encoder_t RightEncoder;

volatile int32_t pulse_left = 0;
volatile float left_rad_raw = 0.0f;
volatile float left_vel_raw = 0.0f;

volatile int32_t pulse_right = 0;
volatile float right_rad_raw = 0.0f;
volatile float right_vel_raw = 0.0f;

/*=========================== PI_VEL PARAMETER =========================*/
volatile uint8_t pid_trigger = 0;

// Left side velocity control
float Kp_L = 560.0f;
float Ki_L = 6000.0f;
float Kb_L = 0.0f;
float error_sat_p_L = 0.0f;
float error_sat_L   = 0.0f;
float ui_p_L        = 0.0f;
volatile float setpoint_L      = 0.0f;
volatile int16_t motor_L_output = 0;

// Right side velocity control
float Kp_R = 470.0f;
float Ki_R = 5000.0f;
float Kb_R = 0.0f;
float error_sat_p_R = 0.0f;
float error_sat_R   = 0.0f;
float ui_p_R        = 0.0f;
volatile float setpoint_R      = 0.0f;
volatile int16_t motor_R_output = 0;

/*=========================== UART & ODOMETRY ==========================*/
// Received data from Pi
uint8_t rxFrame[10];
float linear_x  = 0.0f;
float angular_z = 0.0f;
volatile uint16_t uart_timeout_counter = 0;

// Transmitted Odometry data to Pi
uint8_t txFrame[14];
float x     = 0.0f;
float y     = 0.0f;
float theta = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void encoder_process(void);
void PI_vel(float rad_L, float rad_R);
void Set_Motor(int16_t left_output, int16_t right_output);
void AMR_Teleop_Process(void);
void UART1_Send_Odom(float x_pos, float y_pos, float theta_pos);
void Update_Odometry(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*================== CALCULATE ENCODER ======================*/
void encoder_process(void)
{
    Encoder_Update(&LeftEncoder);
    Encoder_Update(&RightEncoder);

    pulse_left    = -LeftEncoder.encoder_position;
    left_rad_raw  = -LeftEncoder.speed_rad_s;
    left_vel_raw  = -LeftEncoder.velocity_m_s;

    pulse_right   = -RightEncoder.encoder_position;
    right_rad_raw = -RightEncoder.speed_rad_s;
    right_vel_raw = -RightEncoder.velocity_m_s;
}

/*=================== PI VELOCITY CONTROL ===================*/
void PI_vel(float rad_L, float rad_R)
{
    static uint8_t was_running_L = 0;
    static uint8_t was_running_R = 0;

    // ==================== 1. LEFT WHEEL ====================
    if (fabsf(setpoint_L) > 0.05f)
    {
        if (!was_running_L)
        {
            ui_p_L = 0.0f;
            error_sat_p_L = 0.0f;
            was_running_L = 1;
        }

        float error_L   = setpoint_L - rad_L;
        float up_L      = Kp_L * error_L;
        float ui_L      = ui_p_L + Ts * (Ki_L * error_L + Kb_L * error_sat_p_L);
        float out_L_raw = up_L + ui_L;

        if (out_L_raw > PID_LIMIT_L) {
            motor_L_output = PID_LIMIT_L;
            error_sat_L = PID_LIMIT_L - out_L_raw;
        } else if (out_L_raw < -PID_LIMIT_L) {
            motor_L_output = -PID_LIMIT_L;
            error_sat_L = -PID_LIMIT_L - out_L_raw;
        } else {
            motor_L_output = (int16_t)out_L_raw;
            error_sat_L = 0.0f;
        }

        ui_p_L = ui_L;
        error_sat_p_L = error_sat_L;
    }
    else
    {
        motor_L_output = 0;
        ui_p_L = 0.0f;
        error_sat_p_L = 0.0f;
        was_running_L = 0;
    }

    // ==================== 2. RIGHT WHEEL ====================
    if (fabsf(setpoint_R) > 0.05f)
    {
        if (!was_running_R)
        {
            ui_p_R = 0.0f;
            error_sat_p_R = 0.0f;
            was_running_R = 1;
        }

        float error_R   = setpoint_R - rad_R;
        float up_R      = Kp_R * error_R;
        float ui_R      = ui_p_R + Ts * (Ki_R * error_R + Kb_R * error_sat_p_R);
        float out_R_raw = up_R + ui_R;

        if (out_R_raw > PID_LIMIT_R) {
            motor_R_output = PID_LIMIT_R;
            error_sat_R = PID_LIMIT_R - out_R_raw;
        } else if (out_R_raw < -PID_LIMIT_R) {
            motor_R_output = -PID_LIMIT_R;
            error_sat_R = -PID_LIMIT_R - out_R_raw;
        } else {
            motor_R_output = (int16_t)out_R_raw;
            error_sat_R = 0.0f;
        }

        ui_p_R = ui_R;
        error_sat_p_R = error_sat_R;
    }
    else
    {
        motor_R_output = 0;
        ui_p_R = 0.0f;
        error_sat_p_R = 0.0f;
        was_running_R = 0;
    }
}

/*======================== MOTOR CONTROL ========================*/
void Set_Motor(int16_t left_output, int16_t right_output)
{
    if (left_output >= 0) {
        if (left_output > ARR_MAX_L) left_output = ARR_MAX_L;
        TIM8->CCR3 = ARR_MIN;
        TIM8->CCR4 = left_output;
    }
    else {
        left_output = -left_output;
        if (left_output > ARR_MAX_L) left_output = ARR_MAX_L;
        TIM8->CCR3 = left_output;
        TIM8->CCR4 = ARR_MIN;
    }

    if (right_output >= 0) {
        if (right_output > ARR_MAX_R) right_output = ARR_MAX_R;
        TIM4->CCR3 = right_output;
        TIM4->CCR4 = ARR_MIN;
    }
    else {
        right_output = -right_output;
        if (right_output > ARR_MAX_R) right_output = ARR_MAX_R;
        TIM4->CCR3 = ARR_MIN;
        TIM4->CCR4 = right_output;
    }
}

/*==================== PROCESS RECEIVED DATA =====================*/
void AMR_Teleop_Process(void)
{
    uart_timeout_counter += 10;
    if (uart_timeout_counter > 200)
    {
        linear_x  = 0.0f;
        angular_z = 0.0f;
    }

    float v_left_m_s  = linear_x - (angular_z * (Base_Link / 2.0f));
    float v_right_m_s = linear_x + (angular_z * (Base_Link / 2.0f));

    setpoint_L = v_left_m_s / WHEEL_RADIUS;
    setpoint_R = v_right_m_s / WHEEL_RADIUS;
}

/*=========================== RECEIVE DATA (UART1) ===========================*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        __HAL_UNLOCK(&huart1);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, 10);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        if (rxFrame[0] == 0xAA && rxFrame[9] == 0x0D)
        {
            memcpy(&linear_x,  &rxFrame[1], 4);
            memcpy(&angular_z, &rxFrame[5], 4);
            uart_timeout_counter = 0;
        }
    }
}

/*======================= TIMER 5 (10ms) INTERRUPT ========================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        pid_trigger = 1;
    }
}

/*========================= ODOM TRANSMIT (UART1) ======================*/
void UART1_Send_Odom(float x_pos, float y_pos, float theta_pos)
{
    txFrame[0] = 0xAA;

    memcpy(&txFrame[1], &x_pos, 4);
    memcpy(&txFrame[5], &y_pos, 4);
    memcpy(&txFrame[9], &theta_pos, 4);

    txFrame[13] = 0x0D;

    HAL_UART_Transmit(&huart1, txFrame, sizeof(txFrame), 10);
}

/*========================= CALCULATE ODOM ======================*/
void Update_Odometry(void)
{
    theta = Robot_Yaw * DEG_TO_RAD;

    float v_robot = (left_vel_raw + right_vel_raw) / 2.0f;

    x += v_robot * cosf(theta) * Ts;
    y += v_robot * sinf(theta) * Ts;

    while (theta >  3.14159265f)
        theta -= 2.0f * 3.14159265f;
    while (theta < -3.14159265f)
        theta += 2.0f * 3.14159265f;
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  GPIOA->ODR &= ~(1 << 2);   // POWER LED
  GPIOE->ODR &= ~(1 << 1);   // ENABLE MPU6050 (Keo PE1 xuong GND)
  HAL_Delay(100);

  // 1. KHOI TAO VA CAN CHINH MPU6050 TRUC TIEP (Comment khi chi tune PID)
  uint8_t mpu_status = 1;
	for (uint8_t retry = 0; retry < 10; retry++)
	{
			if (MPU6050_Init_Direct() == 0)
			{
					mpu_status = 0;
					break; // Khoi tao thanh cong -> thoat vong lap
			}
			HAL_Delay(50); // Cho them 50ms truoc khi thu lai
	}

	// Neu sau 10 lan van loi moi bao dong coi
	if (mpu_status != 0)
	{
			while (1)
			{
					GPIOE->ODR ^= (1 << 2); // Chop coi bao dong
					HAL_Delay(100);
			}
	}

  // Can chinh tinh Offset Gyro (Giu yen bo mach ~1s tren ban)
  MPU6050_Calibrate_Gyro();
  
  // 2. KHOI TAO PWM DONG CO
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  
  // 3. KHOI TAO ENCODER
  Encoder_Init(&LeftEncoder, TIM2);
  Encoder_Init(&RightEncoder, TIM3);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // 4. BAT NGAT TIMER 5 (10ms chu ky dieu khien)
  HAL_TIM_Base_Start_IT(&htim5);
  
  // 5. BAT NHAN DMA CHO UART TELEOP (Comment khi chi tune PID)
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxFrame, 10);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  
  Kb_L = (Kp_L > 0.001f) ? (Ki_L / Kp_L) : 0.0f;
  Kb_R = (Kp_R > 0.001f) ? (Ki_R / Kp_R) : 0.0f;

  // 6. KEU 3 TIENG BIP BAO SAN SANG
  if (Is_Calibrated == 1)
  {
      for (int i = 0; i < 3; i++) {
          GPIOE->ODR |= (1 << 2);
          HAL_Delay(50);
          GPIOE->ODR &= ~(1 << 2);
          HAL_Delay(50);
      }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Vong dieu khien chay theo ngat TIM5 (10ms)
    if (pid_trigger == 1)
    {
        pid_trigger = 0;

        // 1. Doc cam bien MPU6050 va Encoder
        MPU6050_Read_Accel_Raw();
        IMU_Process_Yaw();
        encoder_process();

        // 2. Cap nhat Odometry va xu ly van toc dat tu Pi
        Update_Odometry();
        AMR_Teleop_Process();

        // 3. Dieu khien PI van toc 2 banh xe
        PI_vel(left_rad_raw, right_rad_raw);
        Set_Motor(motor_L_output, motor_R_output);

        // 4. Gui Odometry len Pi voi chu ky 20ms (50Hz)
        static uint8_t send_odom_counter = 0;
        if (++send_odom_counter >= 2) {
            send_odom_counter = 0;
            UART1_Send_Odom(x, y, theta);
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
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 99;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 7;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 20999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BUZZER_Pin|MPU_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BUZZER_Pin MPU_Enable_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|MPU_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
