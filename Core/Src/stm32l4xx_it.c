/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "display_lcd.h"
#include "i2c-lcd.h"
#include "rotary_encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int A_sig = 0;
int B_sig = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void us_wait(uint32_t us);
extern void ms_wait(uint32_t ms);
extern void write_cur(int);
extern void write_LED(int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern int quick_select;
extern int LED_blink;
extern int current_panel;	// 1 = recoding, 2 = mode select, 3 = setting
extern int cur_select;		//cursor position
extern int current_mode; 	// 0 = normal, 1 = ch1 short recording, 2 = ch2 short recording
extern int CW_mode;		// 0 = normal(pulse), 1 = ch1 continuous wave, 2 = ch2 CW, 3 ch1&2 CW

extern int PWM_delay;		//us  delay between the trigger signal and the first laser pulse
extern int PWM_shift;		//us
extern int PWM_length;		//ms
extern int PWM_length_sub;	//ms  Duration for the second channel in sub channel short recording mode
extern int PWM1_width;		//us
extern int PWM2_width;		//us
extern float PWM1_FQ;		//Hz
extern float PWM2_FQ;		//Hz
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Camera_trigger_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  if(PWM_length == 0){					//exit interrupt with no laser
	  return;
  }

  EXTI->IMR1 = 0xff820000;				//disable EXTI LINE1 and 15

  TIM15->BDTR &= ~TIM_BDTR_MOE;			//stop the foot switch pulses

  TIM2->EGR = TIM_EGR_UG;									//reset timer counter and prescaler counter
  TIM3->EGR = TIM_EGR_UG;

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);		//for checking the delay of GPIO PIN 6
  us_wait(PWM_delay);										//delay from the camera trigger pulse
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

  int PWM1_AAR, PWM2_AAR;

  if(current_mode == 0){
	  PWM1_AAR = PWM_length*10-1;
	  PWM2_AAR = PWM_length*10-1;
  }else if(current_mode == 1){							// This is for short second channel measurement for baseline fluorescence "F"
	  PWM1_AAR = PWM_length_sub*10-1;
	  PWM2_AAR = PWM_length*10-1;
  }else if(current_mode == 2){
	  PWM1_AAR = PWM_length*10-1;
	  PWM2_AAR = PWM_length_sub*10-1;
  }

  TIM6->SR = 0;											//see "General-purpose timer cookbook" p11
  TIM6->PSC = 7999;										//Set timer 6 for duration of the Laser 1
  TIM6->ARR = PWM1_AAR;
  //TIM6->EGR = TIM_EGR_UG;									//reset timer counter and prescaler counter

  TIM6->DIER = TIM_DIER_UIE;								//interrupt enable

  TIM7->SR = 0;
  TIM7->PSC = 7999;										//Set timer 6 for duration of the Laser 2
  TIM7->ARR = PWM2_AAR;
  //TIM7->EGR = TIM_EGR_UG;									//reset timer counter and prescaler counter

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);					//start Laser 1

  TIM6->CR1 |= TIM_CR1_CEN;								//counter enable

  us_wait(PWM_shift);										//shift phase (1ms for 1000Hz recording(500Hz recording for two channel))


  TIM7->DIER = TIM_DIER_UIE;								//interrupt enable

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);					//start Laser 2

  TIM7->CR1 |= TIM_CR1_CEN;								//counter enable

  lcd_clear();
  lcd_put_cur(0,0);
  lcd_send_string ("Recording.....    ");					//Don't put this before recording. It makes long delay to making pulses.

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Rotary_encorder_A_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  //Rotary encoder detection

  EXTI->IMR1 = 0xff820000;				//disable EXTI LINE1 and 15

  A_sig = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_2);  //check pin A status
  B_sig = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_3);  //check pin B status

  if(A_sig == 1){								//rising edge detection
	  if(B_sig == 0){
		  rotary_encorder_forward();
	  } else if(B_sig == 1){
		  rotary_encorder_backward();
	  }
  } else if(A_sig == 0){						//falling edge detection

  }

  EXTI->IMR1 = 0xff8220be;									//enable EXTI LINE1,2,3,4,5,7,13

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Rotary_encorder_B_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  EXTI->IMR1 = 0xff820000;				//disable EXTI LINE1 and 15

  /*  no use, B signal interruption This interruption is needed for accurate encoding.
  //Rotary encoder detection .

  A_sig = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_2);  //check pin A status
  B_sig = HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_3);  //check pin B status

  if(A_sig == 1){
  							//rising edge detection
  } else if(A_sig == 0){						//falling edge detection

  }
*/

  EXTI->IMR1 = 0xff8220be;									//enable EXTI LINE1,2,3,4,5,7,13

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Cursor_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  //push the dial (move cur)

  EXTI->IMR1 = 0xff820000;				//disable EXTI LINE1 and 15
  if(cur_select == 4){
	  cur_select = 1;
	  write_cur(1);

  } else if(cur_select == 1){
	  cur_select = 2;
	  write_cur(2);

  } else if(cur_select == 2){
	  cur_select = 3;
	  write_cur(3);

  } else if(cur_select ==3){
	  cur_select = 4;
	  write_cur(4);
  }
  ms_wait(5);
  EXTI->IMR1 = 0xff8220be;									//enable EXTI LINE1,2,3,4,5,7,13

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(stim_camera_trigger_Pin);
  HAL_GPIO_EXTI_IRQHandler(stim_software_trigger_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  //trigger for stimulator (test)
  EXTI->IMR1 = 0xff820000;				//disable EXTI LINE1 and 15
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

  ms_wait(2000);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  EXTI->IMR1 = 0xff8220be;									//enable EXTI LINE1,2,3,4,5,7,13

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  //quick setting button

  if(quick_select == 4){
	  quick_select = 1;				//default for 1000Hz with 2ch recording(500Hz)

	  LED_blink = 1;
	  current_mode = 0; 	// 0 = normal, 1 = ch1 short recording, 2 = ch2 short recording
	  CW_mode = 0;		// 0 = normal(pulse), 1 = ch1 continuous wave, 2 = ch2 CW, 3 ch1&2 CW

	  PWM_delay = 650;
	  PWM_shift = 1000;	//us
	  PWM_length = 104;	//ms
	  PWM_length_sub = 10; //ms  Duration for 2ch in sub channel short recording mode
	  PWM1_width = 200;				//us
	  PWM2_width = 200;				//us
	  PWM1_FQ = 497.7;	//Hz
	  PWM2_FQ = 497.7;	//Hz

  } else if(quick_select == 1){
	  quick_select = 2;				//1000Hz with 1ch or 2000Hz with 2ch recording

	  LED_blink = 2;
	  current_mode = 0; 	// 0 = normal, 1 = ch1 short recording, 2 = ch2 short recording
	  CW_mode = 0;		// 0 = normal(pulse), 1 = ch1 continuous wave, 2 = ch2 CW, 3 ch1&2 CW

	  PWM_delay = 650;
	  PWM_shift = 500;	//us
	  PWM_length = 204;	//ms
	  PWM_length_sub = 10; //ms  Duration for 2ch in sub channel short recording mode
	  PWM1_width = 200;				//us
	  PWM2_width = 200;				//us
	  PWM1_FQ = 995.9;	//Hz
	  PWM2_FQ = 995.9;	//Hz

  } else if(quick_select == 2){
	  quick_select = 3;				//2000Hz for 1ch

	  LED_blink = 3;
	  current_mode = 0; 	// 0 = normal, 1 = ch1 short recording, 2 = ch2 short recording
	  CW_mode = 0;		// 0 = normal(pulse), 1 = ch1 continuous wave, 2 = ch2 CW, 3 ch1&2 CW

	  PWM_delay = 650;
	  PWM_shift = 500;	//us
	  PWM_length = 204;	//ms
	  PWM_length_sub = 10; //ms  Duration for 2ch in sub channel short recording mode
	  PWM1_width = 200;				//us
	  PWM2_width = 200;				//us
	  PWM1_FQ = 1992;	//Hz
	  PWM2_FQ = 1992;	//Hz

  } else if(quick_select ==3){
	  quick_select = 4;				//short Na pulse mode

	  LED_blink = 4;
	  current_mode = 2; 	// 0 = normal, 1 = ch1 short recording, 2 = ch2 short recording
	  CW_mode = 0;		// 0 = normal(pulse), 1 = ch1 continuous wave, 2 = ch2 CW, 3 ch1&2 CW

	  PWM_delay = 650;
	  PWM_shift = 1000;	//us
	  PWM_length = 104;	//ms
	  PWM_length_sub = 10; //ms  Duration for 2ch in sub channel short recording mode
	  PWM1_width = 200;				//us
	  PWM2_width = 50;				//us
	  PWM1_FQ = 497.7;	//Hz
	  PWM2_FQ = 497.7;	//Hz

  }

  TIM2->ARR = roundf((1/PWM1_FQ)*10000000);		//convert from Hz to ARR(usually it needs -1)
  TIM3->ARR = roundf((1/PWM2_FQ)*10000000);		//convert from Hz to ARR

  TIM2->CCR1 = PWM1_width*10;
  TIM3->CCR1 = PWM2_width*10;

  ms_wait(10);
  write_LCD(1);
  ms_wait(10);
  write_cur(cur_select);
  ms_wait(100);

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC channel1 and channel2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  //Handler for the timer of the Lasers

  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);					//stop Laser 1

  TIM6->CR1 &= ~TIM_CR1_CEN;								//counter disable

  if(current_mode == 2){
	  ms_wait(5000);								//margin for next recording(5s is for preventing wrong 40Hz recording)

	  write_LCD(current_panel);									//show "recording..." in a LCD
	  ms_wait(5);
	  write_cur(cur_select);
	  ms_wait(5);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

	  EXTI->IMR1 = 0xff8220be;									//enable EXTI LINE1,2,3,4,5,7,13
	  TIM15->BDTR |= TIM_BDTR_MOE;							//start the foot switch pulses
  }

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);					//stop Laser 2

  TIM7->CR1 &= ~TIM_CR1_CEN;								//counter disable

  if(current_mode == 0 || current_mode == 1 ){
	  ms_wait(5000);								//margin for next recording(5s is for preventing wrong 40Hz recording)

	  write_LCD(current_panel);									//show "recording..." in a LCD
	  ms_wait(5);
	  write_cur(cur_select);
	  ms_wait(5);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

	  EXTI->IMR1 = 0xff8220be;									//enable EXTI LINE1,2,3,4,5,7,13
	  TIM15->BDTR |= TIM_BDTR_MOE;							//start the foot switch pulses

  }

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
