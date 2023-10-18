/*
 * show data in LCD
 *
 * display_lcd.c
 *
 *  Created on: Jan 30, 2020
 *      Author: Kenichi Miyazaki
 */


#include "display_lcd.h"
#include <stdio.h>
#include "i2c-lcd.h"

extern int current_mode; 	// 0 = normal, 1 = ch1 short recording, 2 = ch2 short recording
int CW_mode;			// 0 = normal(pulse), 1 = ch1 continuous wave, 2 = ch2 CW, 3 ch1&2 CW

extern int PWM_delay;		//us  delay between the trigger signal and the first laser pulse
extern int PWM_shift;		//us
extern int PWM_length;		//ms
extern int PWM_length_sub;	//ms  Duration for 2ch in sub channel short recording mode
extern int PWM1_width;		//us
extern int PWM2_width;		//us
extern int PWM_LED_width;	//us  Foot switch frequency
extern float PWM1_FQ;		//Hz
extern float PWM2_FQ;		//Hz


char *int2char(int num){
	  static char num_char[10]={0};
	  snprintf(num_char, 10," %d", num);
	  return num_char;
}


void write_LCD(int panel){

	lcd_clear();

	if(panel == 1){
		lcd_put_cur(0,0);
		char* PWM1_width_char = int2char(PWM1_width);
		lcd_send_string (PWM1_width_char);
		lcd_send_string ("us");

		lcd_put_cur(0,8);
		char* PWM2_width_char = int2char(PWM2_width);
		lcd_send_string (PWM2_width_char);
		lcd_send_string ("us");

		lcd_put_cur(1,0);
		char* PWM_length_char = int2char(PWM_length);
		lcd_send_string (PWM_length_char);
		lcd_send_string ("ms");

		lcd_put_cur(1,9);
		lcd_send_string ("REC");

	}else if(panel == 2){
		lcd_put_cur(0,1);
		if(current_mode == 0){
			lcd_send_string ("Normal");
		}else if(current_mode == 1){
			lcd_send_string ("ShortC1");
		}else if(current_mode == 2){
			lcd_send_string ("ShortC2");
		}

		lcd_put_cur(0,8);
		char* PWM_length_sub_char = int2char(PWM_length_sub);
		lcd_send_string (PWM_length_sub_char);
		lcd_send_string ("ms");

		lcd_put_cur(1,1);
		if(CW_mode == 0){
			lcd_send_string ("CW None");
		}else if(CW_mode == 1){
			lcd_send_string ("CW Ch1");
		}else if(CW_mode == 2){
			lcd_send_string ("CW Ch2");
		}else if(CW_mode == 3){
			lcd_send_string ("CW Ch12");
		}

		lcd_put_cur(1,9);
		lcd_send_string ("MODE");

	}else if(panel == 3){
			lcd_put_cur(0,0);
			char* PWM_delay_char = int2char(PWM_delay);
			lcd_send_string (PWM_delay_char);
			lcd_send_string ("us");

			lcd_put_cur(0,8);
			char* PWM_shift_char = int2char(PWM_shift);
			lcd_send_string (PWM_shift_char);
			lcd_send_string ("us");

			lcd_put_cur(1,0);
			char* PWM1_FQ_char = int2char(PWM1_FQ*10);					//This is only for PWM1 (usually PWM1 and 2 should be same value)
			lcd_send_string (PWM1_FQ_char);
			lcd_send_string ("Hz");

			lcd_put_cur(1,9);
			lcd_send_string ("Setting");
	}else if(panel ==0){
		lcd_put_cur(0,1);
		lcd_send_string ("Width");

		lcd_put_cur(0,8);
		char* PWM_LED_width_char = int2char(PWM_LED_width);
		lcd_send_string (PWM_LED_width_char);
		lcd_send_string ("us");

		lcd_put_cur(1,0);
		lcd_send_string ("-----");

		lcd_put_cur(1,9);
		lcd_send_string ("FootSW");
	}
}

void write_cur(int cur_pos){
	if(cur_pos == 1){
		lcd_put_cur(0,0);
		lcd_send_string (">");
		lcd_put_cur(0,8);
		lcd_send_string (" ");
		lcd_put_cur(1,0);
		lcd_send_string (" ");
		lcd_put_cur(1,8);
		lcd_send_string (" ");
	}
	else if(cur_pos == 2){
		lcd_put_cur(0,0);
		lcd_send_string (" ");
		lcd_put_cur(0,8);
		lcd_send_string (">");
		lcd_put_cur(1,0);
		lcd_send_string (" ");
		lcd_put_cur(1,8);
		lcd_send_string (" ");
	}
	else if(cur_pos == 3){
		lcd_put_cur(0,0);
		lcd_send_string (" ");
		lcd_put_cur(0,8);
		lcd_send_string (" ");
		lcd_put_cur(1,0);
		lcd_send_string (">");
		lcd_put_cur(1,8);
		lcd_send_string (" ");
	}
	else if(cur_pos == 4){
		lcd_put_cur(0,0);
		lcd_send_string (" ");
		lcd_put_cur(0,8);
		lcd_send_string (" ");
		lcd_put_cur(1,0);
		lcd_send_string (" ");
		lcd_put_cur(1,8);
		lcd_send_string (">");
	}
}
