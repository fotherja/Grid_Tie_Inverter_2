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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "support.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define					ADC_OFFSET								250																			// Corrects the offset from the ISO224
#define         SINE_STEPS          			256                         						// Number of steps to build our sinewave in
  
#define					PLL_Kp										1.0e-5f																	// PID parameters for our PLL tracking control
#define					PLL_Ki										2.0e-6f
#define					PLL_Kd										0.0f
#define					PLL_ADJ_RATE_LIMIT				500.0f	
#define					PLL_PERIOD								7.8e-5f																	// 1 / (50 * 256) seconds
#define					SINE_STEP_PERIOD					13125																		// Ticks between incrementing our LO (Local Osc) index for 50Hz sine

#define					I_OUT_Kp									0.018f																	// PID parameters for our current output controller 
#define					I_OUT_Ki									25.0f
#define					I_OUT_Kd									0.0f
#define					I_OUT_Limit								975.0f
#define					I_OUT_PERIOD							1.0e-4f
#define					I_OUT_LOOP_PERIOD					16800																		// Ticks between I_OUT_PID iterations (100us)

#define					DUTY_LIMIT								975				
#define					PROPORTIONALITY_K_A				878																			// 875 is the value that minimises the difference +3 for equilibrium
#define					TARGET_OUTPUT_CURRENT			-100.0f
#define 				START_UP_CURRENT					-10.0f
#define					CURRENT_RAMP_RATE					0.005f

#define					GAIN_SIGMA_DELTA					256
#define					OFFSET_SIGMA_DELTA				32768					

#define					RMS_UPPER_LIMIT						4700000																		// Actually, we don't bother with the root...
#define					RMS_LOWER_LIMIT						4100000
#define					FREQ_UPPER_LIMIT					130																				// Corrosponds to +/- 0.5Hz
#define					FREQ_LOWER_LIMIT				 -130
#define					V_SUPPLY_CUTOUT						3100
#define					V_SUPPLY_MINIMUM					2850																			// The minimum voltage needs to be above the rectified grid voltage
#define					CURRENT_MAX								28000
#define					PID_I_ERROR_TOLERATED			1000
#define					CURRENT_REJOIN_THRESHOLD	30000

#define					RUNNING_MASK_CNT					1000																			// Once running, if out of whack for > 100ms, we shut off
#define					RESTART_MASK_CNT			 	 -1000																			// Monitor mains Vrms for 1sec before considering re-engaging 
#define					GRID_BAD_FAIL_RATE				10																				//
#define					GRID_OK										0	

#define 				CONSTRAIN(x,lower,upper)	((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Sine_LookupF[256] = 
{0.00,6.28,12.56,18.83,25.09,31.34,37.56,43.77,49.94,56.09,62.20,68.28,74.31,80.30,86.24,92.13,
97.97,103.74,109.45,115.10,120.68,126.18,131.61,136.96,142.23,147.41,152.50,157.50,162.40,167.21,171.92,176.52,
181.02,185.41,189.68,193.85,197.89,201.82,205.62,209.30,212.86,216.28,219.58,222.74,225.77,228.67,231.42,234.04,
236.51,238.85,241.04,243.08,244.98,246.73,248.33,249.78,251.08,252.23,253.23,254.07,254.77,255.31,255.69,255.92,
256.00,255.92,255.69,255.31,254.77,254.07,253.23,252.23,251.08,249.78,248.33,246.73,244.98,243.08,241.04,238.85,
236.51,234.04,231.42,228.67,225.77,222.74,219.58,216.28,212.86,209.30,205.62,201.82,197.89,193.85,189.68,185.41,
181.02,176.52,171.92,167.21,162.40,157.50,152.50,147.41,142.23,136.96,131.61,126.18,120.68,115.10,109.45,103.74,
97.97,92.13,86.24,80.30,74.31,68.28,62.20,56.09,49.94,43.77,37.56,31.34,25.09,18.83,12.56,6.28,	
	
0.00,-6.28,-12.56,-18.83,-25.09,-31.34,-37.56,-43.77,-49.94,-56.09,-62.20,-68.28,-74.31,-80.30,-86.24,-92.13,
-97.97,-103.74,-109.45,-115.10,-120.68,-126.18,-131.61,-136.96,-142.23,-147.41,-152.50,-157.50,-162.40,-167.21,-171.92,-176.52,
-181.02,-185.41,-189.68,-193.85,-197.89,-201.82,-205.62,-209.30,-212.86,-216.28,-219.58,-222.74,-225.77,-228.67,-231.42,-234.04,
-236.51,-238.85,-241.04,-243.08,-244.98,-246.73,-248.33,-249.78,-251.08,-252.23,-253.23,-254.07,-254.77,-255.31,-255.69,-255.92,
-256.00,-255.92,-255.69,-255.31,-254.77,-254.07,-253.23,-252.23,-251.08,-249.78,-248.33,-246.73,-244.98,-243.08,-241.04,-238.85,
-236.51,-234.04,-231.42,-228.67,-225.77,-222.74,-219.58,-216.28,-212.86,-209.30,-205.62,-201.82,-197.89,-193.85,-189.68,-185.41,
-181.02,-176.52,-171.92,-167.21,-162.40,-157.50,-152.50,-147.41,-142.23,-136.96,-131.61,-126.18,-120.68,-115.10,-109.45,-103.74,
-97.97,-92.13,-86.24,-80.30,-74.31,-68.28,-62.20,-56.09,-49.94,-43.77,-37.56,-31.34,-25.09,-18.83,-12.56,-6.28};
	
static uint8_t								PDM_Raw_Data[64];																		// 2 * 32 buffers. Decimate by 256. 5.25Mhz -> 20.5KHz
static volatile uint8_t				Data_to_Process = 0;

static volatile int32_t				Measured_I;																					// Stores our most up to date output current reading 16bit value	
static volatile float					Measured_I_f;

static int16_t								ADC_Raw_DC_Bus_V[5];																// Stores ADC readings for the DC_Bus voltages
static int16_t								ADC_Raw_Line_V[5];																	// Likewise stores ADC readings for the AC Line voltage
static int16_t								ADC_Line_P[5];
static int16_t								ADC_Line_N[5];
static int32_t 								V_Bus, V_Line;													

static PIDControl 						I_OUT_PID;																					
static PIDControl							PLL_PID;
	
static float									I_Output_Demand = TARGET_OUTPUT_CURRENT;

static uint16_t								Sine_Index = 0;

static volatile int32_t				Mains_Good_Bad_Counter = RESTART_MASK_CNT;
static volatile int8_t				HB_DISABLED = true, REQUEST_JOIN = false;

static volatile uint32_t 			Mains_RMS, I_Out_RMS_Error;
static volatile int16_t 			Freq_Offset;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);		
void HB_Disable(void);
void HB_Enable(void); 
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC3_Init();
  MX_TIM9_Init();
  MX_DAC_Init();
  MX_SPI3_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */	
	
	// 1) Start the ADC engines - These continuously replenish these circular buffers with fresh ADC readings
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Raw_DC_Bus_V, 5);	
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_Line_N, 5);
		HAL_ADC_Start_DMA(&hadc3, (uint32_t*)ADC_Line_P, 5);
		
	// 2) Configure our PID parameters
		PIDInit(&I_OUT_PID, I_OUT_Kp, I_OUT_Ki, I_OUT_Kd, I_OUT_PERIOD, -I_OUT_Limit, I_OUT_Limit, AUTOMATIC, DIRECT);
		PIDInit(&PLL_PID, PLL_Kp, PLL_Ki, PLL_Kd, PLL_PERIOD, -PLL_ADJ_RATE_LIMIT, PLL_ADJ_RATE_LIMIT, AUTOMATIC, DIRECT);
		PIDSetpointSet(&PLL_PID, 0.0);
		
	// 3) Start receiving PDM from the AMC1306. This gets processed in 32 byte blocks as a double buffer system
		HAL_SPI_Receive_DMA(&hspi3, PDM_Raw_Data, 64);
		
	// 4) Start our DAC and our timers
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		HAL_TIM_Base_Start_IT(&htim9);
		HAL_TIM_Base_Start_IT(&htim10);
		HAL_TIM_Base_Start_IT(&htim11);	
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(Data_to_Process)																														// When a buffer is filled this is set (to either 32 or 64)
		{		
			static int32_t I_1, I_2, I_3, D_1, D_2, D_3;											
			static int32_t Previous_Input, Previous_D_1, Previous_D_2;
			
			uint8_t x;
			uint8_t Stop_Byte_Adr = Data_to_Process;
			uint8_t Start_Byte_Adr = Stop_Byte_Adr - 32;
			
			Data_to_Process = 0;																												// Clear the flag now we've started processing
						
			for(uint8_t byte_index = Start_Byte_Adr; byte_index < Stop_Byte_Adr; byte_index++)	{
				x = (PDM_Raw_Data[byte_index] >> 7) & 1;
				I_1 = I_1 + x; I_2 = I_2 + I_1;	I_3 = I_3 + I_2;
				x = (PDM_Raw_Data[byte_index] >> 6) & 1;
				I_1 = I_1 + x; I_2 = I_2 + I_1;	I_3 = I_3 + I_2;
				x = (PDM_Raw_Data[byte_index] >> 5) & 1;
				I_1 = I_1 + x; I_2 = I_2 + I_1; I_3 = I_3 + I_2;				
				x = (PDM_Raw_Data[byte_index] >> 4) & 1;
				I_1 = I_1 + x; I_2 = I_2 + I_1; I_3 = I_3 + I_2;
				x = (PDM_Raw_Data[byte_index] >> 3) & 1;
				I_1 = I_1 + x; I_2 = I_2 + I_1; I_3 = I_3 + I_2;
				x = (PDM_Raw_Data[byte_index] >> 2) & 1;
				I_1 = I_1 + x; I_2 = I_2 + I_1; I_3 = I_3 + I_2;
				x = (PDM_Raw_Data[byte_index] >> 1) & 1;
				I_1 = I_1 + x; I_2 = I_2 + I_1; I_3 = I_3 + I_2;				
				x = (PDM_Raw_Data[byte_index] >> 0) & 1;
				I_1 = I_1 + x; I_2 = I_2 + I_1; I_3 = I_3 + I_2;
			}
			
			D_1 = I_3 - Previous_Input;
			Previous_Input = I_3;
			
			D_2 = D_1 - Previous_D_1;
			Previous_D_1 = D_1;
			
			D_3 = D_2 - Previous_D_2;
			Previous_D_2 = D_2;		

			Measured_I = OFFSET_SIGMA_DELTA - (D_3 / GAIN_SIGMA_DELTA);
			Measured_I_f = (float)Measured_I;
		}
		
		// Process incoming ADC Data		
		V_Bus = Get_Median(ADC_Raw_DC_Bus_V, 5, 0);	

		ADC_Raw_Line_V[0] = ADC_Line_P[0] - ADC_Line_N[0];
		ADC_Raw_Line_V[1] = ADC_Line_P[1] - ADC_Line_N[1];
		ADC_Raw_Line_V[2] = ADC_Line_P[2] - ADC_Line_N[2];
		ADC_Raw_Line_V[3] = ADC_Line_P[3] - ADC_Line_N[3];
		ADC_Raw_Line_V[4] = ADC_Line_P[4] - ADC_Line_N[4];
		
		V_Line = Get_Median(ADC_Raw_Line_V, 5, ADC_OFFSET);		
		
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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


/* USER CODE BEGIN 4 */
void HB_Disable() {
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

	HB_DISABLED = true;
}

void HB_Enable() {
	// Start our PWM driver which uses Timer1 to power our H-bridge. Both channels start with Duty = 0
	I_OUT_PID.iTerm = 0;
	I_OUT_PID.setpoint = 0;
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	HB_DISABLED = false;
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
	Data_to_Process = 32;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {	
	Data_to_Process = 64;	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	{	
	// -----------------------------------------------------------------------------------------------------------------------------------
  // #################### Runs every 1ms to perform our grid/DC Bus checks etc for our anti-islanding ##################################
	// -----------------------------------------------------------------------------------------------------------------------------------
	if (htim == &htim9)
  {
			Mains_RMS = Integrate_Mains_RMS(V_Line);																			// Update our mains RMS measurement
			Freq_Offset = (int16_t)PLL_PID.output;																				// Get our current phase shift between our LO and the grid

			if(Mains_RMS > RMS_UPPER_LIMIT || Mains_RMS < RMS_LOWER_LIMIT)								// If mains RMS voltage whacky
				Mains_Good_Bad_Counter -= GRID_BAD_FAIL_RATE;		
			
			if(Freq_Offset > FREQ_UPPER_LIMIT || Freq_Offset < FREQ_LOWER_LIMIT) 					// If mains phase is whacky
				Mains_Good_Bad_Counter -= GRID_BAD_FAIL_RATE;
			
			if(V_Bus < V_SUPPLY_MINIMUM || V_Bus > V_SUPPLY_CUTOUT) 											// If our supply voltage is too low/high cutout immediately
				Mains_Good_Bad_Counter = RESTART_MASK_CNT;			

			if(HB_DISABLED == false)	
			{
				if(Measured_I > CURRENT_MAX || Measured_I < -CURRENT_MAX)										// If our measured current is too large, cutout.
					Mains_Good_Bad_Counter -= GRID_BAD_FAIL_RATE;						

				if(I_Out_RMS_Error > PID_I_ERROR_TOLERATED)																						// If our current control is not adaquate											
					Mains_Good_Bad_Counter -= GRID_BAD_FAIL_RATE;	
			}			
					
	// -----------------------------------------------------------------------------------------------------------------------------------
			if(Mains_Good_Bad_Counter < RUNNING_MASK_CNT)																	// Slowly zero any error counts
				Mains_Good_Bad_Counter++;
			
			if(Mains_Good_Bad_Counter < RESTART_MASK_CNT)																	// Limit the minimum the Good_Bad counter can go
				Mains_Good_Bad_Counter = RESTART_MASK_CNT;
			
			if(Mains_Good_Bad_Counter < GRID_OK && HB_DISABLED == false)	{								// If our metrics have been whacky for too long stop output
				HB_Disable();																																// This puts the H-bridge into a high impedance state
				Mains_Good_Bad_Counter = RESTART_MASK_CNT;
			}

			if(Mains_Good_Bad_Counter == RUNNING_MASK_CNT && HB_DISABLED == true)	{				// With our output idle, if the grid has normalised, restart
				I_Output_Demand = START_UP_CURRENT;	
				REQUEST_JOIN = true;
			}
			
			if(I_Output_Demand > TARGET_OUTPUT_CURRENT)																		// Slowly ramp our output current to setpoint
				I_Output_Demand -= CURRENT_RAMP_RATE;		
	}

	// -----------------------------------------------------------------------------------------------------------------------------------
  // ######### Runs every 100us to measure the instananeous current output and adjust our PWM duty to reach setpoint current ###########
	// -----------------------------------------------------------------------------------------------------------------------------------
  if (htim == &htim10)
  {
		if(REQUEST_JOIN == true)	{																											// Rejoin at a ZCP if not outputing yet
			if(Sine_Index == 0)	{
				REQUEST_JOIN = false;
				HB_Enable();				
			}
		}				
 
		int32_t Duty_Feedforward = (PROPORTIONALITY_K_A * V_Line) / V_Bus;
				
		// Whilst our current sensor is shut down just use the feedforward controller
		if(Measured_I > CURRENT_REJOIN_THRESHOLD)	{
			I_OUT_PID.output = 0;
			I_OUT_PID.error = 0;
		}		
		else	{
			PIDSetpointSet(&I_OUT_PID, Sine_LookupF[Sine_Index] * I_Output_Demand);	
			PIDInputSet(&I_OUT_PID, Measured_I_f);																				
			PIDCompute(&I_OUT_PID);			
		}
			
		// Add together our Feedforward and PID parts
		int16_t Duty_Cycle = (int16_t)Duty_Feedforward + (int16_t)PIDOutputGet(&I_OUT_PID);
		
		Duty_Cycle = CONSTRAIN(Duty_Cycle, -DUTY_LIMIT, DUTY_LIMIT);

		if(Duty_Cycle >= 0)	{																													
			htim1.Instance->CCR1 = Duty_Cycle;
			htim1.Instance->CCR2 = 0;							
		}
		else	{
			htim1.Instance->CCR1 = 0;
			htim1.Instance->CCR2 = -Duty_Cycle;
		}		

		// Write to the DAC so we can debug
		uint32_t DAC_Data = (uint32_t)((Measured_I/16) + 2048);	
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Data);    
  }
	
	// -----------------------------------------------------------------------------------------------------------------------------------
	// ################### Keeps our local oscillator synchronised to the grid by implimenting a PLL with PI controller ##################
	// -----------------------------------------------------------------------------------------------------------------------------------
	if (htim == &htim11)
	{	
		uint16_t Sine_Index_PS 	= (Sine_Index + 64) % 256;														// Our lookup index for a 90 degree phase shifted LO value 	
		int32_t Signal_Multiple = (int32_t)Sine_LookupF[Sine_Index_PS] * V_Line;			// Multiply the phase shifted LO value with our current Line sample		
		PLL_PID.input 					= (float)Integral(Signal_Multiple);										// Integrate this Multiple over the last 1 period	

		PIDCompute(&PLL_PID);																													// Plug result in a PI controller to maintain 0 phase shift		
		TIM11->ARR 							= SINE_STEP_PERIOD - (int32_t)PLL_PID.output;					// adjust LO frequency (step period) to synchronise to mains phase	
		if(++Sine_Index >= SINE_STEPS)	{Sine_Index = 0;}															// Increment our LO index and rollover if need be

		I_Out_RMS_Error 				= Integrate_I_Error((int32_t)I_OUT_PID.error);
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
