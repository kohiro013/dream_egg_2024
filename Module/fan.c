#include "module.h"

#define AUTORELOAD 	(__HAL_TIM_GET_AUTORELOAD(&htim8))
#define DUTY_MAX	(1.f)
#define DUTY_MIN	(0.1f)

#define MOT_SET_COMPARE_SUCTION(x)	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, x)
#define MOT_SET_COMPARE_CLEANER(x)	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, x)

/* ---------------------------------------------------------------
	ファン用の動作周波数とDuty比を設定する関数
--------------------------------------------------------------- */
void Fan_Initialize( void )
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

/* ---------------------------------------------------------------
	吸引ファンを回転させる関数
--------------------------------------------------------------- */
void SuctionFan_Start( float duty )
{
	if( duty > DUTY_MAX ) {
		duty = DUTY_MAX;
	} else if( duty < DUTY_MIN ) {
		duty = DUTY_MIN;
	} else;

	for( float i = 0.1; i <= duty; i += 0.1f ) {
		MOT_SET_COMPARE_SUCTION((uint32_t)(AUTORELOAD * i) - 1);
		HAL_Delay(10);
	}
}

/* ---------------------------------------------------------------
	吸引ファンの回転を停止する関数
--------------------------------------------------------------- */
void SuctionFan_Stop( void )
{
	MOT_SET_COMPARE_SUCTION(0);
}

/* ---------------------------------------------------------------
	掃除ファンを回転させる関数
--------------------------------------------------------------- */
void CleanerFan_Start( float duty )
{
	if( duty > DUTY_MAX ) {
		duty = DUTY_MAX;
	} else if( duty < DUTY_MIN ) {
		duty = DUTY_MIN;
	} else;

	for( float i = 0.1; i <= duty; i += 0.1f ) {
		MOT_SET_COMPARE_CLEANER((uint32_t)(AUTORELOAD * i) - 1);
		HAL_Delay(10);
	}
}

/* ---------------------------------------------------------------
	吸引ファンの回転を停止する関数
--------------------------------------------------------------- */
void CleanerFan_Stop( void )
{
	MOT_SET_COMPARE_CLEANER(0);
}
