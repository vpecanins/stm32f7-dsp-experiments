/**
  ******************************************************************************
  * @file    BSP/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    22-April-2016 
  * @brief   This example code shows how to use the STM32746G Discovery BSP Drivers
  * 
  * AUDIO INFO:
  * 
  * Sample Rate:          48 KHz (defined in stm32746g_discovery_audio.h)
  * Bits per mono sample: 16 bits (signed short)
  * Number of channels:   2 (stereo)
  * Block size:           65534 mono samples (0xFFFE)
  * Samples per block:    32767 stereo samples
  * Block buffer length:  131068 bytes
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include <stdio.h>
#include "string.h"
#include "arm_math.h" 

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

typedef enum
{
  BUFFER_OFFSET_NONE = 0,
  BUFFER_OFFSET_HALF = 1,
  BUFFER_OFFSET_FULL = 2,
}BUFFER_StateTypeDef;

typedef struct {
	int xmin;
	int xmax;
	int ymin;
	int ymax;
} GL_Chart;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static uint16_t  internal_buffer[AUDIO_BLOCK_SIZE];
uint32_t  block_number;
uint8_t   current_screen;

GL_Chart gl_chart = {
	.xmin=-10,
	.xmax=10,
	.ymin=-5,
	.ymax=5
};

/* Global extern variables ---------------------------------------------------*/

uint32_t  audio_rec_buffer_state;

#ifndef USE_FULL_ASSERT
uint32_t    ErrorCounter = 0;
#endif

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void CPU_CACHE_Enable(void);
static void AudioRec_demo (void);

void GL_DrawAxis(GL_Chart* g);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t  lcd_status = LCD_OK;
  
  CPU_CACHE_Enable();
  /* STM32F7xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  /* Configure the system clock to 200 Mhz */
  SystemClock_Config();

  BSP_LED_Init(LED1);

  /* Configure the User Button in EXTI Mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /*##-1- Initialize the LCD #################################################*/
  /* Initialize the LCD */
  current_screen = 1;
  lcd_status = BSP_LCD_Init();
  ASSERT(lcd_status != LCD_OK);
  
  /* LCD Layer 0 Initialization */
  BSP_LCD_LayerDefaultInit(0, LCD_FB0_START_ADDRESS); 
  BSP_LCD_SelectLayer(0);
  BSP_LCD_SetTransparency(0, 0xFF);
  
  /* LCD Layer 1 Initialization */
  BSP_LCD_LayerDefaultInit(1, LCD_FB1_START_ADDRESS); 
  BSP_LCD_SelectLayer(1);
  BSP_LCD_SetTransparency(1, 0x00);
  
  /* LCD Layer 2 Initialization */
  BSP_LCD_LayerDefaultInit(2, LCD_FB2_START_ADDRESS); 
  BSP_LCD_SelectLayer(2);
  BSP_LCD_SetTransparency(2, 0x00);
  
  BSP_LCD_SelectLayer(1);
  
  /* Draw a test circle and background to the layer 1 */
  BSP_LCD_Clear(LCD_COLOR_TRANSPARENT);
  //BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  //BSP_LCD_DrawCircle(BSP_LCD_GetXSize()/2, BSP_LCD_GetYSize()/2, 10);
  GL_DrawAxis(&gl_chart);
  
  /* Return to layer 0 for putting the console */
  BSP_LCD_SelectLayer(0);
  
  /* Enable the display */
  BSP_LCD_DisplayOn();
  
  /* Initialize the LCD Log module */
  LCD_LOG_Init();
  LCD_LOG_SetHeader((uint8_t *)"STM32F7 Audio DSP: Stereo Oscilloscope");
  LCD_DbgLog("LCD log module started\n"); 
  LCD_DbgLog("Press B1 User button to switch screen\n"); 
  LCD_DbgLog("Compiled at: " __DATE__ ", " __TIME__ "\n");

  HAL_Delay(500);

  /* Wait For User inputs */
  AudioRec_demo();
  
  /* Stop recorder */
  BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
  
  LCD_UsrLog("Recording done!\n");
  
  while(1);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 8
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  HAL_StatusTypeDef ret = HAL_OK;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  ASSERT(ret != HAL_OK);

  /* activate the OverDrive to reach the 180 Mhz Frequency */
  ret = HAL_PWREx_ActivateOverDrive();
  ASSERT(ret != HAL_OK);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
  ASSERT(ret != HAL_OK);
}

/**
  * @brief  Check for user input.
  * @param  None
  * @retval Input state (1 : active / 0 : Inactive)
  */
uint8_t CheckForUserInput(void)
{
  if (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    HAL_Delay(10);
    while (BSP_PB_GetState(BUTTON_KEY) != RESET);
    return 1 ;
  }
  return 0;
}

/**
  * @brief  Toggle Leds.
  * @param  None
  * @retval None
  */
void Toggle_Leds(void)
{
  static uint32_t ticks = 0;

  if (ticks++ > 200)
  {
    BSP_LED_Toggle(LED1);
    ticks = 0;
  }
}


/**
  * @brief EXTI line detection callbacks.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{ 
  if(GPIO_Pin==WAKEUP_BUTTON_PIN) 
  { 
    if (current_screen==0) { 
		BSP_LCD_SetTransparency(0, 0xFF); 
		BSP_LCD_SetTransparency(1, 0x00); 
		BSP_LCD_SetTransparency(2, 0x00);
		current_screen=1; 
	} else { 
		BSP_LCD_SetTransparency(0, 0x00); 
		BSP_LCD_SetTransparency(1, 0xFF);
		BSP_LCD_SetTransparency(2, 0xFF); 
		current_screen=0; 
	}
  } 
} 

/**
  * @brief  CPU L1-Cache enable.
  *         Invalidate Data cache before enabling
  *         Enable Data & Instruction Cache
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  (*(uint32_t *) 0xE000ED94) &= ~0x5;
  (*(uint32_t *) 0xE000ED98) = 0x0; //MPU->RNR
  (*(uint32_t *) 0xE000ED9C) = 0x20010000 |1<<4; //MPU->RBAR
  (*(uint32_t *) 0xE000EDA0) = 0<<28 | 3 <<24 | 0<<19 | 0<<18 | 1<<17 | 0<<16 | 0<<8 | 30<<1 | 1<<0 ; //MPU->RASE  WT
  (*(uint32_t *) 0xE000ED94) = 0x5;

  /* Invalidate I-Cache : ICIALLU register*/
  SCB_InvalidateICache();	
	
  /* Enable branch prediction */
  SCB->CCR |= (1 <<18); 
  __DSB();

  /* Enable I-Cache */
  SCB_EnableICache();	
	
  /* Enable D-Cache */
  SCB_InvalidateDCache();
  SCB_EnableDCache();
}

/**
  * @brief  Audio Play demo
  * @param  None
  * @retval None
  */
static void AudioRec_demo (void)
{
  /* Initialize Audio Recorder */
  if (BSP_AUDIO_IN_Init(INPUT_DEVICE_DIGITAL_MICROPHONE_2, 100, DEFAULT_AUDIO_IN_FREQ) == AUDIO_OK)
  {
    LCD_UsrLog("BSP Audio Init OK\n");
  }
  else
  {
    LCD_UsrLog("BSP Audio Init ERROR!\n");
  }

  audio_rec_buffer_state = BUFFER_OFFSET_NONE;

  /* Display the state on the screen */ 
  LCD_UsrLog("Recording...\n");

  /* Start Recording */
  BSP_AUDIO_IN_Record(internal_buffer, AUDIO_BLOCK_SIZE);

  while (1)
  {
    /* Wait end of half block recording */
    while(audio_rec_buffer_state != BUFFER_OFFSET_HALF)
    {
	  HAL_Delay(10);
    }
    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
	
    /* Copy recorded 1st half block in SDRAM */
	unsigned int i=0;
	for (i=0; i<AUDIO_BLOCK_SIZE/4; i++) {
		((int16_t *)(AUDIO_REC_START_ADDR_R))[i] = internal_buffer[2*i];
		((int16_t *)(AUDIO_REC_START_ADDR_L))[i] = internal_buffer[2*i+1];
	}

    /* Wait end of one block recording */
    while(audio_rec_buffer_state != BUFFER_OFFSET_FULL)
    {
	  HAL_Delay(10);
    }
    audio_rec_buffer_state = BUFFER_OFFSET_NONE;
	
    /* Copy recorded 2nd half block in SDRAM */
	for (i=AUDIO_BLOCK_SIZE/4; i<AUDIO_BLOCK_SIZE/2; i++) {
		((int16_t *)(AUDIO_REC_START_ADDR_R))[i] = internal_buffer[2*i];
		((int16_t *)(AUDIO_REC_START_ADDR_L))[i] = internal_buffer[2*i+1];
	}
	
    /* Calculate RMS values */
    q15_t rms_l, rms_r;

	arm_rms_q15(
	  ((int16_t *)(AUDIO_REC_START_ADDR_L)),
	  AUDIO_BLOCK_SIZE/2,
	  &rms_l);
	  
	arm_rms_q15(
	  ((int16_t *)(AUDIO_REC_START_ADDR_R)),
	  AUDIO_BLOCK_SIZE/2,
	  &rms_r);
	  
	printf("Block %lu transfer complete (L=%hd, R=%hd)\n", block_number, rms_l, rms_r);
	
    block_number++; 
  }

}

/*------------------------------------------------------------------------------
       Callbacks implementation:
           the callbacks API are defined __weak in the stm32746g_discovery_audio.c file
           and their implementation should be done the user code if they are needed.
           Below some examples of callback implementations.
  ----------------------------------------------------------------------------*/
/**
  * @brief Manages the DMA Transfer complete interrupt.
  * @param None
  * @retval None
  */
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
  audio_rec_buffer_state = BUFFER_OFFSET_FULL;
  return;
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
  audio_rec_buffer_state = BUFFER_OFFSET_HALF;
  return;
}

/**
  * @brief  Audio IN Error callback function.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_IN_Error_CallBack(void)
{
  /* This function is called when an Interrupt due to transfer error on or peripheral
     error occurs. */
  /* Display message on the LCD screen */
  BSP_LCD_SetBackColor(LCD_COLOR_RED);
  BSP_LCD_DisplayStringAt(0, LINE(14), (uint8_t *)"       DMA  ERROR     ", CENTER_MODE);

  /* Stop the program with an infinite loop */
  while (BSP_PB_GetState(BUTTON_KEY) != RESET)
  {
    return;
  }
  /* could also generate a system reset to recover from the error */
  /* .... */
}


void GL_DrawAxis(GL_Chart* g) {
	int i=0;

	int xsize = BSP_LCD_GetXSize();
	int ysize = BSP_LCD_GetYSize();
	
	BSP_LCD_SetTextColor(LCD_COLOR_GRAY);
	
	int nx = g->xmax - g->xmin;
	int ny = g->ymax - g->ymin;
	
	int xstep = xsize / nx;
	int ystep = ysize / ny;
	
	int lcx= -xstep * g->xmin;
	int lcy= -ystep * g->ymin;

	/*Draw vertical lines*/
	for (i=0; i<=nx; i++) {
		BSP_LCD_DrawLine(i==nx ? i*xstep-1 : i*xstep, 
						 i==-g->xmin ? 0 : lcy-5, 
						 i==nx ? i*xstep-1 : i*xstep, 
						 i==-g->xmin ? ysize : lcy+5);
	}
	
	/*Draw horizontal lines*/
	for (i=0; i<=ny; i++) {
		BSP_LCD_DrawLine(i==-g->ymin ? 0 : lcx-5, 
						 i==ny ? i*ystep : i*ystep,
						 i==-g->ymin ? xsize : lcx+5,
						 i==ny ? i*ystep : i*ystep);
	}
};

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
