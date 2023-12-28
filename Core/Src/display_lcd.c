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
extern int CW_mode;			// 0 = normal(pulse), 1 = ch1 continuous wave, 2 = ch2 CW, 3 ch1&2 CW

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

	HD44780_Clear();

	if(panel == 1){
		HD44780_SetCursor(0,0);
		char* PWM1_width_char = int2char(PWM1_width);
		HD44780_PrintStr(PWM1_width_char);
		HD44780_PrintStr("us");

		HD44780_SetCursor(8,0);
		char* PWM2_width_char = int2char(PWM2_width);
		HD44780_PrintStr(PWM2_width_char);
		HD44780_PrintStr("us");

		HD44780_SetCursor(0,1);
		char* PWM_length_char = int2char(PWM_length);
		HD44780_PrintStr(PWM_length_char);
		HD44780_PrintStr("ms");

		HD44780_SetCursor(9,1);
		HD44780_PrintStr("REC");

	}else if(panel == 2){
		HD44780_SetCursor(1,0);
		if(current_mode == 0){
			HD44780_PrintStr("Normal");
		}else if(current_mode == 1){
			HD44780_PrintStr("ShortC1");
		}else if(current_mode == 2){
			HD44780_PrintStr("ShortC2");
		}

		HD44780_SetCursor(8,0);
		char* PWM_length_sub_char = int2char(PWM_length_sub);
		HD44780_PrintStr(PWM_length_sub_char);
		HD44780_PrintStr("ms");

		HD44780_SetCursor(1,1);
		if(CW_mode == 0){
			HD44780_PrintStr("CW None");
		}else if(CW_mode == 1){
			HD44780_PrintStr("CW Ch1");
		}else if(CW_mode == 2){
			HD44780_PrintStr("CW Ch2");
		}else if(CW_mode == 3){
			HD44780_PrintStr("CW Ch12");
		}

		HD44780_SetCursor(9,1);
		HD44780_PrintStr("MODE");

	}else if(panel == 3){
			HD44780_SetCursor(0,0);
			char* PWM_delay_char = int2char(PWM_delay);
			HD44780_PrintStr(PWM_delay_char);
			HD44780_PrintStr("us");

			HD44780_SetCursor(8,0);
			char* PWM_shift_char = int2char(PWM_shift);
			HD44780_PrintStr(PWM_shift_char);
			HD44780_PrintStr("us");

			HD44780_SetCursor(0,1);
			char* PWM1_FQ_char = int2char(PWM1_FQ*10);					//This is only for PWM1 (usually PWM1 and 2 should be same value)
			HD44780_PrintStr(PWM1_FQ_char);
			HD44780_PrintStr("Hz");

			HD44780_SetCursor(9,1);
			HD44780_PrintStr("Setting");
	}else if(panel ==0){
		HD44780_SetCursor(1,0);
		HD44780_PrintStr("Width");

		HD44780_SetCursor(8,0);
		char* PWM_LED_width_char = int2char(PWM_LED_width);
		HD44780_PrintStr(PWM_LED_width_char);
		HD44780_PrintStr("us");

		HD44780_SetCursor(0,1);
		HD44780_PrintStr("-----");

		HD44780_SetCursor(9,1);
		HD44780_PrintStr("FootSW");
	}
}

void write_cur(int cur_pos){
	if(cur_pos == 1){
		HD44780_SetCursor(0,0);
		HD44780_PrintStr(">");
		HD44780_SetCursor(8,0);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(0,1);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(8,1);
		HD44780_PrintStr(" ");
	}
	else if(cur_pos == 2){
		HD44780_SetCursor(0,0);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(8,0);
		HD44780_PrintStr(">");
		HD44780_SetCursor(0,1);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(8,1);
		HD44780_PrintStr(" ");
	}
	else if(cur_pos == 3){
		HD44780_SetCursor(0,0);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(8,0);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(0,1);
		HD44780_PrintStr(">");
		HD44780_SetCursor(8,1);
		HD44780_PrintStr(" ");
	}
	else if(cur_pos == 4){
		HD44780_SetCursor(0,0);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(8,0);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(0,1);
		HD44780_PrintStr(" ");
		HD44780_SetCursor(8,1);
		HD44780_PrintStr(">");
	}
}
