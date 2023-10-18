/*
 * rotary_encoder.c
 *
 *  Created on: Sep 26, 2020
 *      Author: Kenichi Miyazaki
 */

#include <stdio.h>
#include "main.h"
#include "display_lcd.h"
#include "math.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim15;

extern int current_panel;	// 1 = recoding, 2 = mode select, 3 = setting, 4 = LED Foot switch
extern int cur_select;		//cursor position
extern int current_mode; 	// 0 = normal, 1 = ch1 short recording, 2 = ch2 short recording
extern int CW_mode;		// 0 = normal(pulse), 1 = ch1 continuous wave, 2 = ch2 CW, 3 ch1&2 CW

extern int PWM_delay;		//us  delay between the trigger signal and the first laser pulse
extern int PWM_shift;		//us
extern int PWM_length;		//ms
extern int PWM_length_sub;	//ms  Duration for the second channel in sub channel short recording mode
extern int PWM1_width;		//us
extern int PWM2_width;		//us
extern int PWM_LED_width;	//us  foot switch frequency
extern float PWM1_FQ;		//Hz
extern float PWM2_FQ;		//Hz

extern void ms_wait(uint32_t ms);

void rotary_encorder_forward(void)  // right turn
{
	if(cur_select == 4){

		if(current_panel == 0){
			current_panel = 1;
		} else if(current_panel == 1){
			current_panel = 2;
		} else if(current_panel == 2){
			current_panel = 3;
		}

	}else{
		if(current_panel == 1){

			if(cur_select == 1){
				if(PWM1_width >= 100 && PWM1_width < 1000){								//for Laser 1
					PWM1_width = PWM1_width + 100;
				} else if(PWM1_width == 1000){
					PWM1_width = 5000;								//it make just only one long pulse in 1000Hz
				} else if(PWM1_width > 1000){
//					PWM1_width = 0;									// go back to 0 from 5000 : disable
				} else if(PWM1_width == 0){
					PWM1_width = 50;
				} else if(PWM1_width == 50){
					PWM1_width = 100;
				}
				TIM2->CCR1 = PWM1_width*10;

			} else if(cur_select == 2){								//for Laser 2
				if(PWM2_width >= 100 && PWM2_width < 1000){
					PWM2_width = PWM2_width + 100;
				} else if(PWM2_width == 1000){
					PWM2_width = 5000;								//it make just only one long pulse in 1000Hz
				} else if(PWM2_width > 1000){
//					PWM2_width = 0;									// go back to 0 from 5000 : disable
				} else if(PWM2_width == 0){
					PWM2_width = 50;
				} else if(PWM2_width == 50){
					PWM2_width = 100;
				}
				TIM3->CCR1 = PWM2_width*10;

			} else if(cur_select ==3){								//for recording duration
				if(PWM_length == 0){
					PWM_length = 44;
				} else if(PWM_length == 44){
					PWM_length = 104;
				} else if(PWM_length >= 104 && PWM_length < 1000){
					PWM_length = PWM_length + 100;
				} else if(PWM_length >= 1000){
//					PWM_length = 0;									// go back to 0 from 1000 : disable
				}
			}
		}else if(current_panel == 2){									//mode settings

			if(cur_select == 1){
				if(current_mode == 0){
					current_mode = 1;
					PWM1_width = 200;				//us
					PWM2_width = 200;				//us
				} else if(current_mode == 1){
					current_mode = 2;
				}
			} else if(cur_select == 2){
				if(PWM_length_sub > 0 && PWM_length_sub < PWM_length-10){		//increase sub channel duration
					PWM_length_sub = PWM_length_sub + 10;
				}
			} else if(cur_select == 3){
				// for CW laser mode, Dial right turn
				if(CW_mode == 0){
					CW_mode = 1;
					TIM2->CCR1 = roundf((1/PWM1_FQ)*10000000);					//CW
					TIM3->CCR1 = PWM2_width*10;
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);					//start Laser 1
					HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);					//stop Laser 2
				}else if(CW_mode == 1){
					CW_mode = 2;
					TIM3->CCR1 = roundf((1/PWM1_FQ)*10000000);					//CW
					TIM2->CCR1 = PWM1_width*10;
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);					//start Laser 2
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);					//stop Laser 1
				}else if (CW_mode == 2){
					CW_mode = 3;
					TIM2->CCR1 = roundf((1/PWM1_FQ)*10000000);					//CW
					TIM3->CCR1 = roundf((1/PWM1_FQ)*10000000);					//CW
					HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);					//start Laser 1
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);					//start Laser 2
				}
			}


		}else if(current_panel == 3){

			if(cur_select == 1){										//for PWM delay between trigger and PWM
				PWM_delay = PWM_delay + 1;

			} else if(cur_select == 2){								//PWM phase shift
				PWM_shift = PWM_shift + 1;

			} else if(cur_select ==3){								//PWM 1 and 2 FQ
				PWM1_FQ = PWM1_FQ + 0.1;								//This value is not perfect because of float(ex:498.100006) : this make skipping same values.
				PWM2_FQ = PWM2_FQ + 0.1;
				TIM2->ARR = roundf((1/PWM1_FQ)*10000000);		//convert from Hz to ARR(usually it needs -1)
				TIM3->ARR = roundf((1/PWM2_FQ)*10000000);		//convert from Hz to ARR
			}

		}else if(current_panel == 0){
			if(cur_select == 1 || 2 || 3){
				if(PWM_LED_width >= 100){
					PWM_LED_width = PWM_LED_width + 100;
				} else if(PWM_LED_width >= 10 && PWM_LED_width < 100){
					PWM_LED_width = PWM_LED_width + 10;
				}else if(PWM_LED_width >= 0 && PWM_LED_width < 10){
					PWM_LED_width = PWM_LED_width + 1;
				}
				TIM15->CCR2 = PWM_LED_width*10;								//set width
			}
		}
	}

	write_LCD(current_panel);
	ms_wait(1);
	write_cur(cur_select);
	ms_wait(1);
}

void rotary_encorder_backward(void)  // left turn
{
	  if(cur_select == 4){

		  if(current_panel == 3){
			  current_panel = 2;
		  } else if(current_panel == 2){
			  current_panel = 1;
		  } else if(current_panel == 1){
			  current_panel = 0;
		  }

	  }else{
		  if(current_panel == 1){

			  if(cur_select == 1){
				  if((PWM1_width <= 1000) & (PWM1_width > 100)){			//for Laser 1
					  PWM1_width = PWM1_width - 100;
				  } else if(PWM1_width == 100){
					  PWM1_width = 50;
				  } else if(PWM1_width == 50){
					  PWM1_width = 0;
				  } else if(PWM1_width <= 0){
//					  PWM1_width = 5000;								//it make just only one long pulse in 1000Hz : disable in the left turn
				  } else if(PWM1_width > 1000){
					  PWM1_width = 1000;
				  }
				  TIM2->CCR1 = PWM1_width*10;

			  } else if(cur_select == 2){								//for Laser 2
				  if((PWM2_width <= 1000) & (PWM2_width > 100)){
					  PWM2_width = PWM2_width - 100;
				  } else if(PWM2_width == 100){
					  PWM2_width = 50;
				  } else if(PWM2_width == 50){
					  PWM2_width = 0;
				  } else if(PWM2_width <= 0){
//					  PWM2_width = 5000;								//it make just only one long pulse in 1000Hz : disable in the left turn
				  } else if(PWM2_width > 1000){
					  PWM2_width = 1000;
				  }
				  TIM3->CCR1 = PWM2_width*10;

			  } else if(cur_select == 3){								//for recording duration
				  if(PWM_length > 104){
					  PWM_length = PWM_length - 100;
				  } else if(PWM_length == 104){
					  PWM_length = 44;
				  } else if(PWM_length == 44){
					  PWM_length = 0;
				  }
			  }
		  }else if(current_panel == 2){									//mode settings

			  if(cur_select == 1){
				  if(current_mode == 2){
					  current_mode = 1;
				  } else if(current_mode == 1){
					  current_mode = 0;
				  }
			  } else if(cur_select == 2){

				  if(PWM_length_sub > 10 && PWM_length_sub < PWM_length){		//decrease sub channel duration
					  PWM_length_sub = PWM_length_sub - 10;
				  }

			  } else if(cur_select == 3){
				  // for CW laser mode, Dial left turn
				  if(CW_mode == 3){
					  CW_mode = 2;
					  TIM3->CCR1 = roundf((1/PWM1_FQ)*10000000);					//CW
					  TIM2->CCR1 = PWM1_width*10;
					  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);					//start Laser 2
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);					//stop Laser 1
				  }else if(CW_mode == 2){
					  CW_mode = 1;
					  TIM2->CCR1 = roundf((1/PWM1_FQ)*10000000);					//CW
					  TIM3->CCR1 = PWM2_width*10;
					  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);					//start Laser 1
					  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);					//stop Laser 2
				  }else if (CW_mode == 1){
					  CW_mode = 0;
					  TIM2->CCR1 = PWM1_width*10;
					  TIM3->CCR1 = PWM2_width*10;
					  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);					//stop Laser 1
					  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);					//stop Laser 2
				  }
			  }

		  }else if(current_panel == 3){

				  if(cur_select == 1){										//for PWM delay between trigger and PWM
					  PWM_delay = PWM_delay - 1;

				  } else if(cur_select == 2){								//PWM phase shift
					  PWM_shift = PWM_shift - 1;

				  } else if(cur_select ==3){								//PWM 1 and 2 FQ
					  PWM1_FQ = PWM1_FQ - 0.1;								//This value is not perfect because of float(ex:498.100006)
					  PWM2_FQ = PWM2_FQ - 0.1;
					  TIM2->ARR = roundf((1/PWM1_FQ)*10000000);		//convert from Hz to ARR(usually it needs -1)
					  TIM3->ARR = roundf((1/PWM2_FQ)*10000000);		//convert from Hz to ARR
			      }
		  }else if(current_panel == 0){
				if(cur_select == 1 || 2 || 3){
					if(PWM_LED_width > 100){
						PWM_LED_width = PWM_LED_width - 100;
					} else if(PWM_LED_width > 10 && PWM_LED_width <= 100){
						PWM_LED_width = PWM_LED_width - 10;
					}else if(PWM_LED_width > 0 && PWM_LED_width <= 10){
						PWM_LED_width = PWM_LED_width - 1;
					}
					TIM15->CCR2 = PWM_LED_width*10;								//set width
				}
		  }
	  }

	  write_LCD(current_panel);
	  ms_wait(1);
	  write_cur(cur_select);
	  ms_wait(1);
}
