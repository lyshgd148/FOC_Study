/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "motor_runtime_param.h"
#include "global_def.h"
#include "conf.h"
#include "foc.h"
#include "arm_math.h"
#include "math.h"
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_pwm_duty(float d_u, float d_v, float d_w)
{
  d_u = min(d_u, 0.9);
  d_v = min(d_v, 0.9);
  d_w = min(d_w, 0.9);
  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (1 - d_u) * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (1 - d_v) * htim1.Instance->ARR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (1 - d_w) * htim1.Instance->ARR);
  __enable_irq();
}

void DWT_Delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - start) < cycles)
    ;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */
  struct Frame
  {
    float data[3];
    unsigned char tail[4];
  };
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /*中断版本------------------------------------------------------------------*/
  LL_GPIO_ResetOutputPin(SPI1_CS_GPIO_Port, SPI1_CS_Pin); // 1️⃣ 拉低 CS
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);          // 2️⃣ 配置 DMA
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 2);
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, 2);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
                         LL_SPI_DMA_GetRegAddr(SPI1),
                         (uint32_t)KTH7823_rx_data,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
                         (uint32_t)KTH7823_tx_data,
                         LL_SPI_DMA_GetRegAddr(SPI1),
                         LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);   // 打开接收中断
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3); // 使能接收通道
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4); // 使能发送通道

  LL_SPI_EnableDMAReq_TX(SPI1); // 4️⃣ 启动 SPI DMA 请求
  LL_SPI_EnableDMAReq_RX(SPI1);
  LL_SPI_Enable(SPI1);
  /*中断版本------------------------------------------------------------------*/

  // LL_SPI_Enable(SPI1);
  // LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
  // LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
  // LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3,
  //                        LL_SPI_DMA_GetRegAddr(SPI1),
  //                        (uint32_t)KTH7823_rx_data,
  //                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  // LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4,
  //                        (uint32_t)KTH7823_tx_data,
  //                        LL_SPI_DMA_GetRegAddr(SPI1),
  //                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  HAL_GPIO_WritePin(SD1_GPIO_Port, SD1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SD2_GPIO_Port, SD2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SD3_GPIO_Port, SD3_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  // set_pwm_duty(0.5, 0, 0);
  rotor_zero_angle = 0.11; // 0.12
  // HAL_Delay(500);
  // set_pwm_duty(0, 0, 0);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_Delay(100);

  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_InjectedStart_IT(&hadc2);

  set_motor_pid(
      0.18, 0, 2,
      0.2, 0, 0,
      0.2, 0.004, 0,
      0.2, 0.004, 0);

  motor_control_context.torque_norm_d = 0;
  motor_control_context.torque_norm_q = 0;
  motor_control_context.type = control_type_torque;

  // motor_control_context.speed = 10;
  // motor_control_context.type = control_type_speed;

  // motor_control_context.position = deg2rad(0); // 上电时的角度当作0度
  // motor_control_context.type = control_type_position;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // bool direction = true;  //(1)阶跃波

  // // 使能 DWT 外设
  // CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; //(2)正弦波 (4)(5) 阻尼顺滑模式
  // DWT->CYCCNT = 0;                                // 清零计数
  // DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 启动

  // // FOC 棘轮旋钮  kp=0.5
  // uint8_t num = 32; //(3) 棘轮旋钮
  // float target = 2 * PI / num;
  // uint8_t select = 0;

  // FOC 旋钮顺滑模式      //(4) 旋钮顺滑模式

  // FOC 旋钮阻尼模式      //(5) 旋钮阻尼模式

  //(6)边界旋钮 //kp=0.18
  // float diff = 2 * PI / 3;
  // float R = -diff / 2;
  // float L = diff / 2;

  // uint8_t num = 64;
  // float target = 2 * PI / num;
  // int8_t select = 0;

  //(7) 弹簧效果
  float target = 0;
  float T;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // if (direction)                               //(1) 阶跃波 (力矩)
    //   motor_control_context.torque_norm_q = 0.4;
    // else
    //   motor_control_context.torque_norm_q = -0.4;
    // HAL_Delay(250);
    // direction = !direction;

    // for (int i = 0; i < 360; i++)                  //(2)正弦波 (力矩)
    // {
    //   motor_control_context.torque_norm_q = 0.4 * arm_sin_f32(deg2rad(i));
    //   // HAL_Delay(2);        // sin-test1
    //   DWT_Delay_us(500);      //sin-test2
    // }

    // while (motor_logic_angle < 0)        //(3) 棘轮旋钮
    // {
    //   motor_logic_angle += 2 * PI;
    // }
    // motor_logic_angle = fmod(motor_logic_angle, 2 * PI);
    // select = (uint8_t)roundf(motor_logic_angle / target);
    // motor_control_context.position = select * target;
    // HAL_Delay(10);

    //(4)旋钮顺滑模式  (力矩)
    // motor_control_context.torque_norm_q = 0.4 * motor_speed / 100;
    // DWT_Delay_us(800);

    //(5)旋钮阻尼模式 (力矩)
    // motor_control_context.torque_norm_q = -0.4 * motor_speed / 100;
    // HAL_Delay(1);

    // //(6)边界旋钮
    // if (R <= motor_logic_angle && motor_logic_angle <= L)
    // {
    //   // motor_control_context.torque_norm_d = 0;
    //   // motor_control_context.torque_norm_q = 0;
    //   // motor_control_context.type = control_type_torque;
    //   // HAL_Delay(10);
    //   float temp = motor_logic_angle / target;
    //   if (temp > 0)
    //     select = floorf(temp + 0.5f);
    //   else if (temp < 0)
    //   {
    //     select = floorf(fabs(temp) + 0.5f);
    //     select = -1 * select;
    //   }
    //   motor_control_context.position = select * target;
    //   HAL_Delay(10);
    // }
    // else
    // {
    //   // motor_control_context.type = control_type_position;
    //   if (motor_logic_angle > L)
    //     motor_control_context.position = L - PI / 18;
    //   else if (motor_logic_angle < R)
    //     motor_control_context.position = R + PI / 24;
    //   // motor_control_context.position = 0;
    //   HAL_Delay(15);
    // }

    //(7)弹簧效果
    T = -(motor_logic_angle - target) / 1.2 - 0.6 * motor_speed / 40;
    motor_control_context.torque_norm_q = T;
    HAL_Delay(8);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();
  /* Wait till HSE is ready */
  while (LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i != 0; i--)
    ;

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

  /* Update the time base */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
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
