#include "module.h"

#define AUTORELOAD		(__HAL_TIM_GET_AUTORELOAD(&htim4))
#define PSC				((&htim4)->Instance->PSC + 1)
#define MOT_DUTY_MIN	(10)		// モータの最低Duty
#define MOT_DUTY_MAX	(500)		// モータの最大Duty

// モータの向き設定
#define MOT_SET_COMPARE_R_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, x)
#define MOT_SET_COMPARE_R_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, x)
#define MOT_SET_COMPARE_L_FORWARD(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, x)
#define MOT_SET_COMPARE_L_REVERSE(x)	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, x)

/* ---------------------------------------------------------------
	モータ用のタイマーを開始する関数
--------------------------------------------------------------- */
void Motor_Initialize( void )
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
}

/* ---------------------------------------------------------------
	モータのの回転を止める関数
--------------------------------------------------------------- */
void Motor_StopPWM( void )
{
	MOT_SET_COMPARE_L_FORWARD( 0xffff );
	MOT_SET_COMPARE_L_REVERSE( 0xffff );
	MOT_SET_COMPARE_R_FORWARD( 0xffff );
	MOT_SET_COMPARE_R_REVERSE( 0xffff );
}

void Motor_BrakePWM( void )
{
	MOT_SET_COMPARE_R_FORWARD( 0 );
	MOT_SET_COMPARE_R_REVERSE( 0 );
	MOT_SET_COMPARE_L_FORWARD( 0 );
	MOT_SET_COMPARE_L_REVERSE( 0 );
}

/* ---------------------------------------------------------------
	左モータを指定のDuty（0～1000）で回転させる関数
--------------------------------------------------------------- */
void Motor_SetDuty_Left( int16_t duty_l )
{
	uint32_t	pulse_l;

	if( ABS(duty_l) > MOT_DUTY_MAX ) {
		pulse_l = (uint32_t)(AUTORELOAD * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_l) < MOT_DUTY_MIN ) {
		pulse_l = (uint32_t)(AUTORELOAD * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_l = (uint32_t)(AUTORELOAD * ABS(duty_l) / 1000) - 1;
	}

	if( duty_l > 0 ) {
		MOT_SET_COMPARE_L_FORWARD( pulse_l );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	} else if( duty_l < 0 ) {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( pulse_l );
	} else {
		MOT_SET_COMPARE_L_FORWARD( 0 );
		MOT_SET_COMPARE_L_REVERSE( 0 );
	}
}

/* ---------------------------------------------------------------
	右モータを指定のDuty（0～1000）で回転させる関数
--------------------------------------------------------------- */
void Motor_SetDuty_Right( int16_t duty_r )
{
	uint32_t	pulse_r;

	if( ABS(duty_r) > MOT_DUTY_MAX ) {
		pulse_r = (uint32_t)(AUTORELOAD * MOT_DUTY_MAX / 1000) - 1;
	} else if( ABS(duty_r) < MOT_DUTY_MIN ) {
		pulse_r = (uint32_t)(AUTORELOAD * MOT_DUTY_MIN / 1000) - 1;
	} else {
		pulse_r = (uint32_t)(AUTORELOAD * ABS(duty_r) / 1000) - 1;
	}

	if( duty_r > 0 ) {
		MOT_SET_COMPARE_R_FORWARD( pulse_r );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	} else if( duty_r < 0 ) {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( pulse_r );
	} else {
		MOT_SET_COMPARE_R_FORWARD( 0 );
		MOT_SET_COMPARE_R_REVERSE( 0 );
	}
}
