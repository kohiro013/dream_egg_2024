#include "motion.h"

#define SLIP_RATE 		(0.02f)
#define SLIP_DISTANCE	(15.f)

// グローバル変数
volatile static float 	cycle_const, cycle_slip;
volatile static float 	cycle_accel, amplitude_accel;
volatile static float 	cycle_slow, amplitude_slow;

/* ---------------------------------------------------------------
	直線走行用パラメータ設定関数
--------------------------------------------------------------- */
void Motion_SetStraightParameters( float acceleration, float deceleration, float max_v, float terminal_v, float distance )
{
	float initial_v = ABS(Vehicle_GetVelocity());
	float distance_accel;
	float distance_slow;

	// 設定距離が0以下の場合の処理
	if( distance < 0 ) {
		cycle_accel = cycle_const = cycle_slow = cycle_slip = 0.f;
		amplitude_accel = amplitude_slow = 0.f;
		return;
	} else;

	// 計算時間中に走行した分の距離を引く
	distance -= ABS(Vehicle_GetDistance());

	// 加速・減速距離の計算
	if( initial_v > max_v ) {
		max_v = initial_v;
	} else;
	distance_accel = (((max_v * max_v) - (initial_v * initial_v)) / (2 * acceleration));
	distance_slow  = (((max_v * max_v) - (terminal_v * terminal_v)) / (2 * deceleration));

	// スリップ距離の計算
	if(terminal_v > 50.f) {
		cycle_slip = MAX(SLIP_DISTANCE, SLIP_RATE * distance) / terminal_v;
		distance -= MAX(SLIP_DISTANCE, SLIP_RATE * distance);
	} else {
		cycle_slip = 0.f;
	}

	// 加速・減速時間の計算
	if( distance < ( distance_accel + distance_slow ) ) {
		cycle_const = 0.0f;
		arm_sqrt_f32(((acceleration * terminal_v * terminal_v) + (deceleration * initial_v * initial_v)
				+ (2 * acceleration * deceleration * distance)) / (acceleration + deceleration), &max_v);
		distance_accel = (((max_v * max_v) - (initial_v * initial_v)) / (2 * acceleration));
		distance_slow = (((max_v * max_v) - (terminal_v * terminal_v)) / (2 * deceleration));
	} else {
		cycle_const	= (distance - distance_accel - distance_slow) / max_v;
	}

	// 加減速用ネイピア関数の周期と振幅の計算
	cycle_accel	= ( max_v - initial_v ) / acceleration;
	if( cycle_accel <= 0 ) amplitude_accel = 0.0f;
	else amplitude_accel = ( max_v - initial_v ) / ( Mynapier_GetIntegral(0) * cycle_accel );

	cycle_slow	= ( max_v - terminal_v ) / deceleration;
	if( cycle_slow <= 0 ) amplitude_slow = 0.0f;
	else amplitude_slow = ( max_v - terminal_v ) / ( Mynapier_GetIntegral(0) * cycle_slow );
}

/* ---------------------------------------------------------------
	直線走行の開始関数
--------------------------------------------------------------- */
void Motion_StartStraight( float acceleration, float deceleration, float max_v, float terminal_v, float distance )
{
	// 直線走行パラメータの設定
	Motion_SetStraightParameters( acceleration, deceleration, max_v, terminal_v, distance );

	// タイマーのリセット
	Vehicle_ResetTimer();
}

/* ---------------------------------------------------------------
	直線走行時間の取得関数
--------------------------------------------------------------- */
float Motion_GetStraightTime( float acceleration, float deceleration, float max_v, float terminal_v, float distance )
{
	Motion_SetStraightParameters( acceleration, deceleration, max_v, terminal_v, distance );
	return cycle_accel + cycle_const + cycle_slow + cycle_slip;
}

/* ---------------------------------------------------------------
	直線の加速度出力関数
--------------------------------------------------------------- */
float Motion_SetStraightAcceleration( float t )
{
	volatile float a = 0.f;

	// 加速時
	if( t < cycle_accel ) {
		a = amplitude_accel * Mynapier_Calc(0, t / cycle_accel);
	// 最高速度時
	} else if( t < cycle_accel + cycle_const ) {
		a = 0.f;
	// 減速時
	} else if( t < cycle_accel + cycle_const + cycle_slow ) {
		a = -amplitude_slow * Mynapier_Calc(0, (t - cycle_accel - cycle_const) / cycle_slow);
	// スリップ調整区間
	} else {
		a = 0.f;
	}
	return a;
}

/* ---------------------------------------------------------------
	直線走行中の待機関数
--------------------------------------------------------------- */
void Motion_WaitStraight( void )
{
	volatile float t = 0.f;

	while( (Control_GetMode() != FAULT) && (t < cycle_accel + cycle_const + cycle_slow + cycle_slip) ) {
		// バッテリの電圧管理
		if( Vehicle_GetVoltage() < Battery_GetLimitVoltage() ) {
			Control_SetMode( FAULT );
		} else {}

		// テレメトリからの中止信号を受信
		if( Telemetry_Decode() == 2 ) {
			Control_SetMode( FAULT );
		} else {}

		// 現在時間の取得
		t = Vehicle_GetTimer();
	}
	cycle_accel = cycle_const = cycle_slow = cycle_slip = 0.f;
	amplitude_accel = amplitude_slow = 0.f;
	Vehicle_SetAcceleration( 0.0f );
	Vehicle_ResetDistance();
	Control_ResetFilterDistance();
}
