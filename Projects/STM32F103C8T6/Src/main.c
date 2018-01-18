/**
  ******************************************************************************
  * @file    HAL/HAL_TimeBase_TIM/Src/main.c 
  * @author  MCD Application Team
  * @version V1.5.0
  * @date    14-April-2017
  * @brief   This example describes how to configure HAL time base using
  *          the STM32F1xx HAL API.
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
#include "u8g2.h"

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup HAL_TimeBase_TIM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED1_PIN                         GPIO_PIN_6
#define LED2_PIN                         GPIO_PIN_7
#define LED_GPIO_PORT                    GPIOB
#define LED_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()  
#define LED_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()  

#define LCD_RST_1       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,  GPIO_PIN_SET)
#define LCD_RST_0       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,  GPIO_PIN_RESET)
#define LCD_RS_1	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define LCD_RS_0	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define LCD_SCLK_1      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define LCD_SCLK_0      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define LCD_SID_1       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET)
#define LCD_SID_0       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static u8g2_t u8g2;

void delay_us(uint32_t time)
{    
   uint32_t i=0;  
	
   while(time--)
   {
      i=10;
      while(i--) ;    
   }
}

void delay_ms(uint32_t time)
{    
   uint32_t i=0;  
	
   while(time--)
   {
      i=12000;
      while(i--) ;    
   }
}
					   
uint8_t u8g2_gpio_and_delay_stm32(U8X8_UNUSED u8x8_t *u8x8, U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int, U8X8_UNUSED void *arg_ptr)
{
		GPIO_InitTypeDef  gpioinitstruct;
	
		switch(msg){

		//Function which implements a delay, arg_int contains the amount of ms
		case U8X8_MSG_GPIO_AND_DELAY_INIT:

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/* Configure the GPIO_LED pin */
		gpioinitstruct.Pin    = GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9;
		gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
		gpioinitstruct.Pull   = GPIO_NOPULL;
		gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &gpioinitstruct);
		
		HAL_GPIO_WritePin(LED_GPIO_PORT, GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_9, GPIO_PIN_SET); 

		break;
		//Function which implements a delay, arg_int contains the amount of ms
		case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;
		//Function which delays 10us
		case U8X8_MSG_DELAY_10MICRO:
		delay_us(10);
		break;
		//Function which delays 100ns
		case U8X8_MSG_DELAY_100NANO:
		__NOP();

		break;
		//Function to define the logic level of the clockline
		case U8X8_MSG_GPIO_SPI_CLOCK:
			if (arg_int) LCD_SCLK_1;
			else LCD_SCLK_0;

		break;
		//Function to define the logic level of the data line to the display
		case U8X8_MSG_GPIO_SPI_DATA:
			if (arg_int) LCD_SID_1;
			else LCD_SID_0;

		break;

		// Function to define the logic level of the CS line
		case U8X8_MSG_GPIO_CS1:
			if (arg_int) LCD_RS_1	;
			else LCD_RS_0;

		break;
		//Function to define the logic level of the Data/ Command line
		case U8X8_MSG_GPIO_DC:

		break;
		//Function to define the logic level of the RESET line
		case U8X8_MSG_GPIO_RESET:
			if (arg_int) LCD_RST_1;
			else LCD_RST_0;

		break;
		
		default:
			return 0; //A message was received which is not implemented, return 0 to indicate an error
	}

	return 1; // command processed successfully.
}

/* Private functions ---------------------------------------------------------*/
void BSP_LED_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct;
  
  /* Enable the GPIO_LED Clock */
  LED_GPIO_CLK_ENABLE();

  /* Configure the GPIO_LED pin */
  gpioinitstruct.Pin    = LED1_PIN|LED2_PIN;
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull   = GPIO_NOPULL;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(LED_GPIO_PORT, &gpioinitstruct);

  /* Reset PIN to switch off the LED */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be de-init. 
  *   This parameter can be one of the following values:
  *     @arg  LED2
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
  */
void BSP_LED_DeInit(void)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = LED1_PIN|LED2_PIN;
  HAL_GPIO_DeInit(LED_GPIO_PORT, gpio_init_structure.Pin);
}


void BSP_LED1_On(void)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_SET); 
}

void BSP_LED1_Off(void)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET); 
}

void BSP_LED1_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED1_PIN);
}

void BSP_LED2_On(void)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, GPIO_PIN_SET); 
}

void BSP_LED2_Off(void)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET); 
}

void BSP_LED2_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED2_PIN);
}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 /* This sample code shows how to configure The HAL time base source base with a 
    dedicated  Tick interrupt priority.
    A general purpose timer (TIM2) is used instead of Systick as source of time base.  
    Time base duration is fixed to 1ms since PPP_TIMEOUT_VALUEs are defined and 
    handled in milliseconds basis.
    */

  /* STM32F1xx HAL library initialization:
       - Configure the Flash prefetch
       - Configure timer (TIM2) to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 72 MHz */
  SystemClock_Config();
  
  /* Configure LED */
  BSP_LED_Init();  
	
	u8g2_Setup_st7920_s_128x64_f(&u8g2, U8G2_R0, u8x8_byte_4wire_sw_spi, u8g2_gpio_and_delay_stm32); // init u8g2 structure
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	
	HAL_Delay(1000);

  /* Insert a Delay of 1000 ms and toggle LED2, in an infinite loop */  
  while (1)
  {
    /* Insert a 1s delay */
    HAL_Delay(1000);  
    BSP_LED2_Toggle();
		BSP_LED1_Toggle();
		
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetFontMode(&u8g2, 1);
		u8g2_SetFontDirection(&u8g2, 0);
		u8g2_SetFont(&u8g2, u8g2_font_helvB18_te);
		u8g2_DrawStr(&u8g2,  0, 24, "hello world");
		u8g2_DrawStr(&u8g2,  0, 50, "i am Re.");
		u8g2_SetFont(&u8g2, u8g2_font_u8glib_4_tf);
		u8g2_DrawStr(&u8g2,  0, 60, "2018-01-18");
		u8g2_SendBuffer(&u8g2);
  }	
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef  ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

#ifdef  USE_FULL_ASSERT
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
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
