#include "application.h"

const float MAX_STRAIGHT = 1500.f;
const float ACC_STRAIGHT = 5000.f;

/* ----------------------------------------------------------------------------------
	最短走行モーション
-----------------------------------------------------------------------------------*/
void Cleaner_StartPathSequence( int8_t is_return )
{
	t_path 	path;

	// エラーかゴールパスに辿り着くまでパスにしたがって走行
	for( uint16_t num = 0; num < 255; num ++ ) {
		if(Control_GetMode() == FAULT) { break; }

		// パスの読み込み
		if(is_return == false) {
			path = Path_GetSequence(num);
		} else {
			path = Path_GetReturnSequence(num);
		}

		// 直進区間
		Control_SetMode(ADJUST);
		Motion_StartStraight(ACC_STRAIGHT, ACC_STRAIGHT, MAX_STRAIGHT, 0.f, SECTION_SIZE * path.straight);
		Motion_WaitStraight();

		// ゴール
		if( path.type == GOAL ) {
			break;
		} else {}

		// ターン区間
		if( path.type == TURN_0 ) {
			if( path.direction == REAR ) {
				Motion_StartRotate(180.f, RIGHT);
			} else {
				Motion_StartRotate(90.f, path.direction);
			}
		} else {
			break;
		}
	}
}

/* ----------------------------------------------------------------------------------
	掃除走行
-----------------------------------------------------------------------------------*/
void Cleaner_Run( int8_t is_return )
{
	// パスがなければ走行させない
	if( Path_GetSequenceNumber() <= 0 ) {
		return;
	} else {}

	Control_SetModeGain( FASTEST );

	// 吸引ファンの起動
	Control_SetMode(ADJUST);
	SuctionFan_Start(0.3f);
//	CleanerFan_Start(0.5f);
	HAL_Delay(200);

	// パスに沿って走行開始
	Cleaner_StartPathSequence( false );

	// 制御のリセット
	SuctionFan_Stop();
	CleanerFan_Stop();
	if(Control_GetMode() != FAULT) {
		HAL_Delay(200);
//		Control_SetMode(NONE);
		Motor_StopPWM();
		LED_TimerBinary(0xff, 100);
		HAL_Delay(1000);
	} else {}

	if(Control_GetMode() != FAULT && is_return == true) {

		// 反転して帰還走行開始
		Motion_StartRotate(180.f, RIGHT);

		Control_ResetGyroDeviation();
		Control_ResetAngleDeviation();

		// テレメトリから帰還信号を受信するまで待機
		Control_SetMode( NONE );
		Motor_StopPWM();
		while( 1 ){
			uint8_t telem = Telemetry_Decode();
			if( telem == 3 ) {
				break;
			// テレメトリからの中止信号を受信で強制終了
			} else if( telem == 2 ) {
				Control_SetMode( FAULT );
				return;
			} else {}
		}

		// 各パラメータのリセット
		Control_ResetFilterDistance();
		IMU_ResetGyroAngle_Z();
		Vehicle_ResetStraight();
		Vehicle_ResetTurning();
		Vehicle_ResetIntegral();
		Control_ResetEncoderDeviation();
		Control_ResetGyroDeviation();
		Control_ResetAngleDeviation();

		// 吸引ファンの起動
		Control_SetMode(ADJUST);
		SuctionFan_Start(0.3f);
//		CleanerFan_Start(0.5f);
		HAL_Delay(200);

		// パスに沿って走行開始
		Cleaner_StartPathSequence( true );

		// 制御のリセット
		SuctionFan_Stop();
		CleanerFan_Stop();
		if(Control_GetMode() != FAULT) {
			// ゴールしたら元の向きに戻す
			Motion_StartRotate(180.f, RIGHT);

			HAL_Delay(200);
			Control_SetMode(NONE);
			Motor_StopPWM();
			LED_TimerBinary(0xff, 100);
			HAL_Delay(1000);
		} else {}
	}
}
