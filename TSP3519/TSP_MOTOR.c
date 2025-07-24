//0723-updated 
#include "ti_msp_dl_config.h"
#include "TSP_MOTOR.h"
#include "tsp_gpio.h"


//limit need to be evaluate

#define CH1_LOWER_LIMIT	100U
#define CH1_UPPER_LIMIT	1900U
#define CH2_LOWER_LIMIT	100U
#define CH2_UPPER_LIMIT	1900U
#define CH3_LOWER_LIMIT	1220U
#define CH3_UPPER_LIMIT	1780U
#define CH4_LOWER_LIMIT	1220U
#define CH4_UPPER_LIMIT	1780U
#define CH5_LOWER_LIMIT	1220U
#define CH5_UPPER_LIMIT	1780U
#define CH6_LOWER_LIMIT	1220U
#define CH6_UPPER_LIMIT	1780U
#define CH7_LOWER_LIMIT	1220U
#define CH7_UPPER_LIMIT	1780U
#define CH8_LOWER_LIMIT	1220U
#define CH8_UPPER_LIMIT	1780U

#define MOTOR_DC_LIMIT	30U



void tsp_servo_angle(uint8_t channel, uint16_t pulse_width)
{
	uint16_t duty;
	
	duty = pulse_width;
	switch (channel)
	{
		case SERVO1:
			if (CH1_LOWER_LIMIT > pulse_width)
				duty = CH1_LOWER_LIMIT;
			if (CH1_UPPER_LIMIT < pulse_width)
				duty = CH1_UPPER_LIMIT;
            DL_Timer_setCaptureCompareValue(SERVO1_2_INST, duty, GPIO_SERVO1_2_C1_IDX);
			break;
		case SERVO2:
			if (CH2_LOWER_LIMIT > pulse_width)
				duty = CH2_LOWER_LIMIT;
			if (CH2_UPPER_LIMIT < pulse_width)
				duty = CH2_UPPER_LIMIT;
            DL_Timer_setCaptureCompareValue(SERVO1_2_INST, duty, GPIO_SERVO1_2_C0_IDX);
			break;
		case SERVO3:
			if (CH3_LOWER_LIMIT > pulse_width)
				duty = CH3_LOWER_LIMIT;
			if (CH3_UPPER_LIMIT < pulse_width)
				duty = CH3_UPPER_LIMIT;
            DL_Timer_setCaptureCompareValue(SERVO3_4_INST, duty, GPIO_SERVO3_4_C1_IDX);
			break;
		case SERVO4:
			if (CH4_LOWER_LIMIT > pulse_width)
				duty = CH4_LOWER_LIMIT;
			if (CH4_UPPER_LIMIT < pulse_width)
				duty = CH4_UPPER_LIMIT;
            DL_Timer_setCaptureCompareValue(SERVO3_4_INST, duty, GPIO_SERVO3_4_C0_IDX);
			break;
		case SERVO5:
			if (CH4_LOWER_LIMIT > pulse_width)
				duty = CH4_LOWER_LIMIT;
			if (CH4_UPPER_LIMIT < pulse_width)
				duty = CH4_UPPER_LIMIT;
            DL_Timer_setCaptureCompareValue(SERVO5_6_INST, duty, GPIO_SERVO5_6_C1_IDX);
			break;
		case SERVO6:
			if (CH4_LOWER_LIMIT > pulse_width)
				duty = CH4_LOWER_LIMIT;
			if (CH4_UPPER_LIMIT < pulse_width)
				duty = CH4_UPPER_LIMIT;
            DL_Timer_setCaptureCompareValue(SERVO5_6_INST, duty, GPIO_SERVO5_6_C0_IDX);
			break;
		case SERVO7:
			if (CH4_LOWER_LIMIT > pulse_width)
				duty = CH4_LOWER_LIMIT;
			if (CH4_UPPER_LIMIT < pulse_width)
				duty = CH4_UPPER_LIMIT;
            DL_Timer_setCaptureCompareValue(SERVO7_8_INST, duty, GPIO_SERVO7_8_C1_IDX);
			break;
		case SERVO8:
			if (CH4_LOWER_LIMIT > pulse_width)
				duty = CH4_LOWER_LIMIT;
			if (CH4_UPPER_LIMIT < pulse_width)
				duty = CH4_UPPER_LIMIT;
            DL_Timer_setCaptureCompareValue(SERVO7_8_INST, duty, GPIO_SERVO1_2_C0_IDX);
			break;
		default:
			break;
	}


}



/*-------------------Test for SERVO----------------------*/

// int main(void)
// {
// 	int16_t Num = 0;
// 	uint16_t pw = 1000;
// 	uint8_t value_ptr[6];
// 	SYSCFG_DL_init();
// 	tsp_Encoder_init();
// 	tsp_tft18_init();




// 	tsp_tft18_clear(BLACK);
// 	tsp_tft18_show_str(16, 0, "Num:");
// 	tsp_tft18_show_str(16, 1, "pw:");



// 	while(1)
// 	{
// 		Num += tsp_Encoder_Get();
// 		pw = 1500 + 10*Num;

// 		tsp_servo_angle(SERVO1, pw);

// 		sprintf(value_ptr, "%05d", Num);
// 		tsp_tft18_show_str(96, 0, value_ptr);
// 		sprintf(value_ptr, "%05d", pw);
// 		tsp_tft18_show_str(96, 1, value_ptr);
// 		delay_1ms(10);
// 	}

// }

/*-------------------Test for SERVO----------------------*/


void tsp_motor_enable()
{
	SLEEP_HIGH();
}

void tsp_motor_disable()
{
	SLEEP_LOW();
}

void tsp_motor_voltage(uint8_t motor_id, uint8_t dir, uint16_t duty_cycle)
{
	uint16_t dc;
	
	dc = duty_cycle;
	if (MOTOR_DC_LIMIT < duty_cycle) dc = MOTOR_DC_LIMIT;
	
	switch(motor_id)
	{
	case MOTOR1:
		switch (dir)
		{
			case FORWARD:
            	DL_Timer_setCaptureCompareValue(MOTOR_INST, dc, GPIO_MOTOR_C1_IDX);     //M1IN1
            	DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C0_IDX);      //M1IN2
				break;
			case BACKWARD:
            	DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C1_IDX);
            	DL_Timer_setCaptureCompareValue(MOTOR_INST, dc, GPIO_MOTOR_C0_IDX);
				break;
			default:
				break;
		}
		break;
	case MOTOR2:
		switch (dir)
		{
			case FORWARD:
            	DL_Timer_setCaptureCompareValue(MOTOR_INST, dc, GPIO_MOTOR_C3_IDX);    //M2IN1
            	DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C2_IDX);     //M2IN2
				break;
			case BACKWARD:
            	DL_Timer_setCaptureCompareValue(MOTOR_INST, 0, GPIO_MOTOR_C3_IDX);
            	DL_Timer_setCaptureCompareValue(MOTOR_INST, dc, GPIO_MOTOR_C2_IDX);
				break;
			default:
				break;
		}
		break;
		
	}
}