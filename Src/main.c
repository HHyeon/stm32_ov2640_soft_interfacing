/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#include "stdio.h"
#include "ov2640_regs.h"
#include "string.h"

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE_PUT int __io_putchar(int ch)
#define PUTCHAR_PROTOTYPE_GET int __io_getchar(int ch)
#else
#define PUTCHAR_PROTOTYPE_PUT int fputc(int ch, FILE *f)
#define PUTCHAR_PROTOTYPE_GET int fgetc(FILE *f)
#endif 

PUTCHAR_PROTOTYPE_PUT
{
  HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFF);
  if(ch == '\n')
  {
    ch = '\r';
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFF);
  }
  return ch;
}

PUTCHAR_PROTOTYPE_GET
{
  uint8_t ch;
  while( HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, 0xFFFF) != HAL_OK);
  return ch;
}


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t timer1_periodelapsed=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == htim1.Instance)
  {
    timer1_periodelapsed = 1;
  }
}

void ov2640_write_byte(uint8_t addr, uint8_t reg)
{
  if(HAL_I2C_Mem_Write(&hi2c1, 0x60, addr,1,&reg,1, 1000) != HAL_OK)
    Error_Handler();
}

void ov2640_read_byte(uint8_t addr, uint8_t *reg)
{
  if(HAL_I2C_Mem_Read(&hi2c1, 0x60, addr,1,reg,1, 1000) != HAL_OK)
    Error_Handler();
}


void ov2640_write_bytes(const struct regval_list *regs)
{
  while((regs->reg_num != 0xff) || (regs->value != 0xff))
  {
    ov2640_write_byte(regs->reg_num, regs->value);
    regs++;
  }
}


void ov2640_allreg_printout()
{
  int len = sizeof(dsp_table_validregaddr);
  ov2640_write_byte(0xFF,0x00);
  printf("DSP reg table 0xFF = 0: \n");
  for(int i=0;i<len;i++)
  {
    uint8_t addr = dsp_table_validregaddr[i];
    uint8_t reg = 0;
    ov2640_read_byte(addr, &reg);
    printf("%2d. \t0x%02x - 0x%02x\n", i+1, addr, reg);
  }
  len = sizeof(sensor_table_validregaddr);
  ov2640_write_byte(0xFF,0x01);
  printf("Sensor reg table 0xFF = 1: \n");
  for(int i=0;i<len;i++)
  {    
    uint8_t addr = sensor_table_validregaddr[i];
    uint8_t reg = 0;
    ov2640_read_byte(addr, &reg);
    printf("%2d. \t0x%02x - 0x%02x\n", i+1, addr, reg);
  }
}

char debugging_string_buffer[256];
uint8_t image_capturing = 0;
uint8_t image_capture_toggle = 0;
uint8_t vsync_valid = 0;
uint8_t href_valid = 0;
uint8_t pclk_valid = 0;
uint8_t vsync_valid_past = 2;
uint8_t href_valid_past = 2;
uint32_t pclkavailability=0;
uint32_t href_cnting=0;
uint32_t href_cnt=0;
uint32_t pclk_cnting=0;
uint32_t pclk_cnt=0;
uint8_t frame_end = 0;
uint8_t frame_sttd= 0;

//uint8_t vsync_changed = 0;
//uint8_t href_changed = 0;

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(PCLK_Pin);
//  HAL_GPIO_EXTI_IRQHandler(HREF_Pin);
//  HAL_GPIO_EXTI_IRQHandler(VSYNC_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == PCLK_Pin)
  {
    pclk_valid=1;
  }
//  else if(GPIO_Pin == HREF_Pin)
//  {
//    href_changed=1;
//  }
//  else if(GPIO_Pin == VSYNC_Pin)
//  {
//    vsync_changed=1;
//  }
}

void ov2640_system_reset()
{
  OV2640_SEN_ENABLE;
  ov2640_write_byte(COM7, COM7_SRST);
  HAL_Delay(100);
}

void ov2640_init(uint8_t reset)
{
  if(reset)
  {
    CAM_RET_HIGH; HAL_Delay(100);
    CAM_PWDN_LOW; HAL_Delay(100);
    ov2640_system_reset();
    
    ov2640_write_bytes(ov2640_init_regs_v2);
    ov2640_write_bytes(ov2640_size_change_preamble_regs);
    ov2640_write_bytes(ov2640_uxga_regs);
//    ov2640_write_bytes(ov2640_format_change_preamble_regs);    
//    ov2640_write_bytes(ov2640_yuyv_regs);
//    ov2640_write_bytes(ov2640_jpeg_regs);
//    OV2640_SEN_ENABLE;
//    uint8_t reg;
//    ov2640_read_byte(REG32, &reg);
//    reg |= 0xC0;
//    ov2640_write_byte(REG32, reg);
  }
  
  ov2640_allreg_printout();

  printf("enable exti\n");
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = PCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PCLK_GPIO_Port, &GPIO_InitStruct);
  
//  GPIO_InitStruct.Pin = HREF_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(PCLK_GPIO_Port, &GPIO_InitStruct);
//  
//  GPIO_InitStruct.Pin = VSYNC_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(PCLK_GPIO_Port, &GPIO_InitStruct);
  
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
}

#define uart3_packet_pdata_len 1600
uint8_t uart3_packet_pdata[uart3_packet_pdata_len];
uint8_t uart3_packet_pdata_txbuf[uart3_packet_pdata_len];
volatile uint8_t uart3_tx_done = 0;
volatile uint8_t uart3_rx_done = 0;
uint8_t uart3_rx_char = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == huart3.Instance)
  {
    uart3_tx_done = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == huart3.Instance)
  {
    uart3_rx_done = 1;
  }
}

void uart3_rx_char_rcv_it()
{
  HAL_UART_Receive_IT(&huart3, &uart3_rx_char, 1);
}

#define jpeg_frame_data_len 4*1024
uint8_t jpeg_frame_data[jpeg_frame_data_len];
uint8_t jpeg_frame_data_txbuf[jpeg_frame_data_len];
uint32_t jpeg_frame_data_index=0;
uint32_t jpeg_frame_data_size_report=0;

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  ov2640_init(1);
  LD_GREEN_HIGH;
  
  uart3_tx_done=1;
  uart3_rx_char_rcv_it();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    if(uart3_rx_done)
    {
      uart3_rx_done=0;
      
      if(uart3_rx_char == 'a')
      {
        image_capture_toggle = 1;
        LD_GREEN_LOW;
      }
      uart3_rx_char_rcv_it();
    }
    
    if(uart3_tx_done)
    {
      uart3_tx_done=0;
    }
    
    if(pclk_valid)
    {
      pclk_valid=0;
      pclkavailability++;
      
      href_valid = HAL_GPIO_ReadPin(HREF_GPIO_Port, HREF_Pin);
      vsync_valid = HAL_GPIO_ReadPin(VSYNC_GPIO_Port, VSYNC_Pin);
      
      if(!vsync_valid)
      {
        if(!href_valid&&href_valid_past) // line ended
        {
          pclk_cnt = pclk_cnting;
          pclk_cnting=0;
          
          if(image_capturing)
          {
            memcpy(uart3_packet_pdata_txbuf, uart3_packet_pdata, sizeof(uart3_packet_pdata_txbuf));
            HAL_UART_Transmit_DMA(&huart3, uart3_packet_pdata_txbuf, uart3_packet_pdata_len);
          }
        }
        else if(href_valid&&!href_valid_past) // line started
        {
          href_cnting++;
        }
        
        if(href_valid) // valid line
        {
          uart3_packet_pdata[pclk_cnting] = GPIOA->IDR & 0xFF;
          if((pclk_cnting)<uart3_packet_pdata_len-1)
            pclk_cnting++;
        }
      }
      
      if(vsync_valid&&!vsync_valid_past) // frame ended
      {
        href_cnt = href_cnting;
        href_cnting=0;
        
        frame_end = 1;
        image_capturing = 0;
        LD_GREEN_HIGH;
      }
      else if(!vsync_valid&&vsync_valid_past) // frame started
      {
        image_capturing = image_capture_toggle; 
        image_capture_toggle = 0; // once erase this line to maintain past action
      }
      
      href_valid_past=href_valid;
      vsync_valid_past=vsync_valid;
    }
    
    if(frame_end)
    {
      frame_end=0;
      
      sprintf(debugging_string_buffer, "\n\rLine Ended:%4d\r\n",href_cnt);
      HAL_UART_Transmit_IT(&huart1, (uint8_t*)debugging_string_buffer, strlen(debugging_string_buffer));
    }
    
    if(timer1_periodelapsed)
    {
      timer1_periodelapsed=0;
      sprintf(debugging_string_buffer, "\rh:%4d,w:%4d:pclk:%5d,%c", href_cnting,pclk_cnt,pclkavailability,image_capturing ? '*' : ' ');
      HAL_UART_Transmit_IT(&huart1, (uint8_t*)debugging_string_buffer, strlen(debugging_string_buffer));
      pclkavailability=0;
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
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
  
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 1000000;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAM_PWDN_Pin|CAM_RET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD_GREEN_Pin */
  GPIO_InitStruct.Pin = LD_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : P0_Pin P1_Pin P2_Pin P3_Pin 
                           P4_Pin P5_Pin P6_Pin P7_Pin */
  GPIO_InitStruct.Pin = P0_Pin|P1_Pin|P2_Pin|P3_Pin 
                          |P4_Pin|P5_Pin|P6_Pin|P7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PCLK_Pin HREF_Pin VSYNC_Pin */
  GPIO_InitStruct.Pin = PCLK_Pin|HREF_Pin|VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CAM_PWDN_Pin CAM_RET_Pin */
  GPIO_InitStruct.Pin = CAM_PWDN_Pin|CAM_RET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  uint8_t n=10;
  CAM_RET_LOW; 
  while(n--)
  {
      HAL_Delay(50);
      HAL_GPIO_TogglePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin);
  }
  CAM_RET_HIGH; 
  printf("stm32 system reset\n");
  NVIC_SystemReset();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
