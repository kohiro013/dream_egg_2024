#include "module.h"

// スイッチ関連マクロ
#define SWITCH_ONOFF()	HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin)	// 押されるとHighが返ってくる

// プッシュスイッチの設定
#define SAMPLING_PERIOD		(3)		// スイッチ押下判定のサンプリング周期[ms]
#define TH_SHORT_PUSH		(5)		// 押下判定のカウント閾値 [count]
#define TH_LONG_PUSH		(300)	// 長押し判定のカウント閾値 [count]
#define TH_RESET_TIME		(2100)	// 押下カウントをリセットする時間閾値[ms]

// 光センサスイッチの設定
#define TH_SENSOR_VALUE		(300)	// 遮光判定の閾値
#define TH_SENSOR_TIME		(300)	// 遮光判定の時間閾値

// TOFセンサスイッチの設定
#define TH_TOF_VALUE		(60)	// 遮光判定の閾値
#define TH_TOF_TIME			(300)	// 遮光判定の時間閾値

// グローバル変数宣言
volatile static uint16_t 	timer 		 = 0;	// サンプリング用タイマー
volatile static uint16_t	push_count 	 = 0;	// スイッチの押下カウント
volatile static int8_t		push_mode 	 = 0;	// スイッチの押下判定
volatile static uint16_t 	sensor_count = 0;	// 光センサの遮光カウント
volatile static int8_t 		sensor_mode  = -1;	// 光センサの遮光判定
volatile static uint16_t 	tof_count 	 = 0;	// TOFセンサの遮光カウント
volatile static int8_t 		tof_mode	 = -1;	// TOFセンサの遮光判定

/* ---------------------------------------------------------------
	割り込み内でスイッチの押下カウントを更新する関数
--------------------------------------------------------------- */
void Switch_Update( void )
{
	// プッシュスイッチのチャタリング対策として、指定のサンプリング周期でスイッチの押下状態を確認し、
	// ONであればそのカウンタがカウントアップされ、ある閾値に達したら押下判定とする
	if( timer % SAMPLING_PERIOD == 0 ) {
		// スイッチの押下カウント
		if( SWITCH_ONOFF() == GPIO_PIN_SET ) {
			push_count ++;
		} else;
	} else;

	timer ++;
	// 一定時間経過したときにスイッチが押されていなければ押下カウンタをリセットする
	if( (timer>= TH_RESET_TIME) && (SWITCH_ONOFF() != GPIO_PIN_SET) ) {
		timer = push_count = 0;
	} else;

	// 前壁センサが反応している時間をカウントする
	if( (Sensor_GetValue(FRONT+LEFT) > TH_SENSOR_VALUE) || (Sensor_GetValue(FRONT+RIGHT) > TH_SENSOR_VALUE) ) {
		if( sensor_mode == -1 ) {
			sensor_count ++;
		} else {
			sensor_count = 0;
		}
	} else {
		sensor_count = 0;
		sensor_mode = -1;
	}

	// TOFセンサが反応している時間をカウントする
	if( TOF_GetDistance(FRONT) < TH_TOF_VALUE ) {
		if( tof_mode == -1 ) {
			tof_count ++;
		} else {
			tof_count = 0;
		}
	} else {
		tof_count = 0;
		tof_mode = -1;
	}
}

/* ---------------------------------------------------------------
	スイッチ押下判定を返す関数(長押し：2　押下判定：1　未押下：0)
--------------------------------------------------------------- */
int8_t Switch_GetIsPush( void )
{
	// 長押し判定
	if( SWITCH_ONOFF() == GPIO_PIN_SET ) {
		if( push_count > TH_LONG_PUSH ) {
			timer = push_count = 0;
			return 2;
		} else {
			push_mode = 0;
		}
	// 押下判定
	} else if( push_count > TH_SHORT_PUSH ) {
		timer = push_count = 0;
		push_mode = 1;
	// 未押下判定
	} else {
		push_mode = 0;
	}

	return push_mode;
}

/* ---------------------------------------------------------------
	光センサの遮光判定を返す関数
--------------------------------------------------------------- */
int8_t Switch_GetIsFrontSensor( void )
{
	if( sensor_count > TH_SENSOR_TIME ) {
		if( (Sensor_GetValue(FRONT+LEFT) > TH_SENSOR_VALUE) && (Sensor_GetValue(FRONT+RIGHT) > TH_SENSOR_VALUE) ) {
			sensor_mode = FRONT;
		} else if( Sensor_GetValue(FRONT+LEFT) > TH_SENSOR_VALUE ) {
			sensor_mode = LEFT;
		} else if( Sensor_GetValue(FRONT+RIGHT) > TH_SENSOR_VALUE ) {
			sensor_mode = RIGHT;
		} else;
	} else {
		sensor_mode = -1;
	}
	return sensor_mode;
}

/* ---------------------------------------------------------------
	TOFセンサの遮光判定を返す関数
--------------------------------------------------------------- */
int8_t Switch_GetIsTOF( void )
{
	if( tof_count > TH_TOF_TIME ) {
		tof_mode = FRONT;
	} else {
		tof_mode = -1;
	}
	return tof_mode;
}
