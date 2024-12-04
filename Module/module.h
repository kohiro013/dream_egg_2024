#pragma once

#include "main.h"
#include "adc.h"
#include "gpdma.h"
#include "i2c.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include <stdint.h>
#include "arm_math.h"

/* 便利な定数群 */
#define SQRT2				(1.41421356237f)			// ルート2
#define SQRT3				(1.73205080757f)			// ルート3
#define SQRT5				(2.2360679775f)				// ルート5
#define SQRT7				(2.64575131106f)			// ルート7
#define _ESC				(0x1b)						// エスケープシーケンス
#define _LF					(0x0a)						// 改行シーケンス
#define _CR					(0x0d)						// 復帰シーケンス

typedef enum {	// Boolean型列挙
	false	= 0,
	true	= 1,
} t_bool;

typedef enum {	// ローカル方向列挙
	RIGHT	= 0,
	FRONT	= 1,
	LEFT	= 2,
	REAR	= 3,
} t_localdir;

typedef enum{	// グローバル方向列挙
	EAST	= 0,
	NORTH	= 1,
	WEST	= 2,
	SOUTH	= 3,
} t_globaldir;

/* 便利なマクロ関数群 */
#define DEG2RAD(x)			(((x)/180.0f)*PI)			// 度数法からラジアンに変換
#define RAD2DEG(x)			(180.0f*((x)/PI))			// ラジアンから度数法に変換
#define SWAP(a, b) 			((a != b) && (a += b, b = a - b, a -= b))
#define ABS(x) 				((x) < 0 ? -(x) : (x))		// 絶対値
#define SIGN(x)				((x) < 0 ? -1 : 1)			// 符号
#define MAX(a, b) 			((a) > (b) ? (a) : (b))		// 2つのうち大きい方を返します
#define MIN(a, b) 			((a) < (b) ? (a) : (b))		// 2つのうち小さい方を返します
#define MAX3(a, b, c) 		((a) > (MAX(b, c)) ? (a) : (MAX(b, c)))
#define MIN3(a, b, c) 		((a) < (MIN(b, c)) ? (a) : (MIN(b, c)))

// UART通信関数群(communication.c)
void 		Communicate_TransmitTest( uint8_t );
uint8_t 	Communicate_ReceiveTest( void );
void 		Communicate_Transmit1byte( uint8_t );
uint8_t 	Communicate_Receive1byte( void );
void 		Communicate_ClearReceiveBuffer( void );

/* LED関数群(led.c) */
void 		LED_Update( void );						// 割り込み内でLEDの点灯時間を更新する
void 		LED_LightBinary( uint8_t );				// バイナリ指定でLEDを点灯させる
void 		LED_ToggleBinary( uint8_t );			// バイナリ指定でLEDをトグル点灯させる
void 		LED_TimerBinary( uint8_t, uint16_t );	// バイナリ指定でLEDを指定時間点滅させる

/* 割り込み関数群(interrupt.c) */
uint16_t 	Interrupt_GetDuty( void );				// 割り込み処理内の計算割合を取得する
uint16_t 	Interrupt_GetDuty_Max( void );			// 割り込み処理内の最大計算割合を取得する
float		Interrupt_GetBootTime( void );			// マイコンが起動してから経過した時間を取得する[s]

/* スイッチ関数群(switch.c) */
void 		Switch_Update( void );					// 割り込み内でスイッチ押下カウントを更新する
int8_t 		Switch_GetIsPush( void );				// スイッチ押下判定を取得する(長押し：2　押下判定：1　未押下：0)
int8_t 		Switch_GetIsFrontSensor( void ); 		// 前壁センサの遮光判定を取得する(左右：1 右のみ：0　左のみ：2　未遮光：-1)
int8_t 		Switch_GetIsTOF( void );

/* エンコーダ関数群(encoder.c) */
void 		Encoder_ResetCount_Left( void );		// 左エンコーダのカウントを初期値にする
void 		Encoder_ResetCount_Right( void );		// 右エンコーダのカウントを初期値にする
float 		Encoder_GetAngle_Left( void );			// 左タイヤの角度を取得する[rad]
float 		Encoder_GetAngle_Right( void );			// 右タイヤの角度を取得する[rad]
void 		Encoder_DebugPrintf( void );

/* 慣性センサ関数群(imu.c) */
uint8_t		IMU_CheckWHOAMI( void );				// 慣性センサの動作確認関数(0x47が返ってくれば正常)
void		IMU_Update( void );						// 慣性センサの値を更新する
void 		IMU_ResetReference( void );				// 慣性センサのリファレンスを補正する
float 		IMU_GetAccel_X( void );					// X軸加速度計の加速度を取得する[m/s^2]
float 		IMU_GetGyro_Z( void );					// Z軸ジャイロの角速度を取得する[rad/s]
float 		IMU_GetGyroAngle_Z( void );				// Z軸ジャイロの角度を取得する[rad]
void 		IMU_SetGyroAngle_Z( float );
void 		IMU_ResetGyroAngle_Z( void );			// Z軸ジャイロの角度をリセットする[rad]
void 		IMU_OffsetGyroAngle_Z( void );
void 		IMU_DebugPrintf( void );

/* 赤外センサ関数群(sensor.c) */
void 		Sensor_TurnOffLED( void );				// 赤外センサのLEDを消灯する
void 		Sensor_TurnOnLED( void );				// 赤外センサのLEDを点灯する
uint16_t 	Sensor_GetBatteryValue( void );			// 電源電圧のAD値を取得する
int16_t 	Sensor_GetValue( uint8_t );				// 赤外センサのLEDオンオフ差分値を取得する
													// 0:横右、1:前右、2:横左、3:前左
void 		Sensor_DebugPrintf( void );

/* バッテリー関数群(battery.c) */
float 		Battery_GetVoltage( void );				// バッテリの電圧を取得する[V]
float 		Battery_GetLimitVoltage( void );
void 		Battery_LimiterVoltage( void );			// バッテリの電圧が制限電圧以下になるとtrueを返す

/* モータ関数群(motor.c) */
void 		Motor_StopPWM( void );					// モータを停止
void 		Motor_BrakePWM( void );					// ショートブレーキ
void 		Motor_SetDuty_Left( int16_t );			// 左モータを指定したDutyで回転させる[0-1000]
void 		Motor_SetDuty_Right( int16_t );			// 右モータを指定したDutyで回転させる[0-1000]

/* モジュールテスト関数群(module_test.c) */
void 		module_test( void );					// 全モジュールの動作確認用テスト関数

/* 吸引ファン関数群(fan.c)	*/
void 		SuctionFan_Start( float );
void 		SuctionFan_Stop( void );
void 		CleanerFan_Start( float );
void 		CleanerFan_Stop( void );

/* TOFセンサ関数群(tof.c)	*/
void 		TOF_Update( void );
int16_t 	TOF_GetDistance( int8_t );
float 		TOF_GetTheta( int8_t );
void 		TOF_DebugPrintf( void );

/* フラッシュ関数群(flash.c) */
uint8_t 	Flash_EraseData( void );				// Flashのデータを消去
void 		Flash_ReadData( uint8_t*, uint32_t );	// Flashのデータを全てに読出し先頭アドレスを取得する
uint8_t 	Flash_WriteData( uint16_t*, size_t );	// Flashにデータを書込む

// NT-Shell関数群(myshell.c)
void 		Myshell_Execute( void );

// テレメトリ関数群
void 		Telemetry_GetData( uint8_t*, uint16_t* );
int8_t 		Telemetry_Decode( void );
void 		Telemetry_Transmit( void );
