
#include "motion.h"
#include "slalom.h"

volatile static int8_t		slalom_type;
volatile static float		turn_v;
volatile static float 		cycle_slalom;
volatile static float		amplitude_slalom;
volatile static float		before_distance;
volatile static float		after_distance;


/* ---------------------------------------------------------------
	スラローム走行の開始関数
--------------------------------------------------------------- */
void Motion_StartSlalom( int8_t type, int8_t direction, int8_t param )
{
	volatile float	angle_slalom;
	volatile float 	radius;

	slalom_type = type;
	if( type == TURN_0 || type == TURN_90 ) {
		param = 0;
	} else if(param >= sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0])) {
		param = sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0]) - 1;
	} else;

	angle_slalom			= DEG2RAD( init_slalom[type-1][param].degree );
	turn_v 					= init_slalom[type-1][param].velocity;
	radius					= init_slalom[type-1][param].radius;

	if( direction == RIGHT ) {
		amplitude_slalom 	= -turn_v / radius;
		before_distance		= init_slalom[type-1][param].before;
		after_distance		= init_slalom[type-1][param].after;
	} else {
		amplitude_slalom 	= turn_v / radius;
		before_distance		= init_slalom[type-1][param].before;
		after_distance		= init_slalom[type-1][param].after;
	}
	cycle_slalom = ( ABS(amplitude_slalom) * Mynapier_GetIntegral(type) ) / angle_slalom;

	if( type == TURN_0 || type == TURN_90 ) {
		Vehicle_ResetTurning();
		Vehicle_ResetIntegral();
		Control_ResetFilterDistance();
//		Control_ResetSensorDeviation();
		IMU_ResetGyroAngle_Z();
		Control_ResetAngleDeviation();
	} else;
	Vehicle_ResetTimer();
}

/* ---------------------------------------------------------------
	スラロームの角加速度出力関数
--------------------------------------------------------------- */
float Motion_SetSlalomAngularAcceleration( float t )
{
	volatile float alpha = 0.f;

	if( t <= before_distance/turn_v ) {
		alpha = 0.f;
	} else if( t < before_distance/turn_v + 1.f/cycle_slalom ) {
		if( t - before_distance/turn_v <= SYSTEM_PERIOD ) {
			alpha = 0.f;
		} else {
			alpha = amplitude_slalom * (Mynapier_Calc(slalom_type, cycle_slalom * (t - before_distance/turn_v))
					- Mynapier_Calc(slalom_type, cycle_slalom * ((t - before_distance/turn_v) - SYSTEM_PERIOD))) / SYSTEM_PERIOD;
		}
	} else {
		alpha = 0.0f;
	}
	return alpha;
}

/* ---------------------------------------------------------------
	スラローム走行中の待機関数
--------------------------------------------------------------- */
void Motion_WaitSlalom( int8_t type, int8_t direction, int8_t param )
{
	volatile float 	t = 0.f;
	// 計算時間中に走行した分の距離を引く
	volatile float 	offset_distance = ABS(Vehicle_GetDistance());

	if( type == TURN_0 || type == TURN_90 ) {
		param = 0;
	} else if(param >= sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0])) {
		param = sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0]) - 1;
	} else;

	// 探索調整用スラローム
	if( type == TURN_0 ) {
		Control_SetMode( TURN );
		Vehicle_SetAcceleration( 0.0f );
		Vehicle_SetVelocity( turn_v );
		Vehicle_ResetTimer();
		while( (Control_GetMode() != FAULT) && (t < (before_distance + after_distance - offset_distance)/turn_v + 1.f/cycle_slalom) ) {
			t = Vehicle_GetTimer();
		}

	// 探索用スラローム
	} else if( type == TURN_90 ) {
		Vehicle_SetAcceleration( 0.0f );
		Vehicle_SetVelocity( turn_v );

		// 各パラメータのリセット
		Vehicle_ResetTurning();
		Vehicle_ResetDistance();
		Control_ResetFilterDistance();
//		Control_ResetSensorDeviation();
		Control_SetMode( TURN );

		// スラローム
		Vehicle_ResetTimer();
		while( (Control_GetMode() != FAULT) && (t < (before_distance + after_distance)/turn_v + 1.f/cycle_slalom) ) {
			t = Vehicle_GetTimer();
		}

		// 各パラメータのリセット
		Vehicle_ResetAngle();
		IMU_ResetGyroAngle_Z();
		Control_ResetAngleDeviation();

	// 通常時のターン
	} else {
		Control_SetMode(TURN);
//		Control_ResetSensorDeviation();

		// 各パラメータのリセット
		Vehicle_ResetTurning();
		Vehicle_ResetDistance();
		Control_ResetFilterDistance();
//		Control_ResetSensorDeviation();
		Control_SetMode( TURN );

		// 前距離
		while( Control_GetMode() != FAULT && Vehicle_GetDistance() < before_distance ) {
			Vehicle_ResetTimer();
		}
		Vehicle_SetAcceleration( 0.f );
		Vehicle_SetVelocity( turn_v );
		Vehicle_ResetDistance();
		Control_ResetFilterDistance();

		// スラローム
		Vehicle_SetTimer(before_distance/turn_v);
		while( Control_GetMode() != FAULT && t < before_distance/turn_v + 1.f/cycle_slalom ) {
			t = Vehicle_GetTimer();
		}
	}

	// 各パラメータのリセット
	Vehicle_ResetTurning();
	Vehicle_ResetDistance();
	Control_ResetFilterDistance();
	cycle_slalom = amplitude_slalom = 0.f;
	before_distance = after_distance = 0.f;
}

int8_t Motion_GetSlalomMaxParameter( void )
{
	return (int8_t)(sizeof(init_slalom[1]) / sizeof(init_slalom[1][0]));
}

/* ----------------------------------------------------------------------------------
	スラロームの時間
-----------------------------------------------------------------------------------*/
float Motion_GetSlalomTime( int8_t type, int8_t param )
{
	if( type == TURN_0 || type == GOAL ) {
		return 0.f;
	} else {
		if(param < sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0])) {
			return init_slalom[type-1][param].time;
		} else {
			return init_slalom[type-1][sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0]) - 1].time;
		}
	}
}

/* ----------------------------------------------------------------------------------
	スラロームの速度
-----------------------------------------------------------------------------------*/
float Motion_GetSlalomVelocity( int8_t type, int8_t param )
{
	if( type == TURN_0 || type == GOAL ) {
		return( 0 );
	} else {
		if(param < sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0])) {
			return init_slalom[type-1][param].velocity;
		} else {
			return init_slalom[type-1][sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0]) - 1].velocity;
		}
	}
}

/* ----------------------------------------------------------------------------------
	スラロームの前距離
-----------------------------------------------------------------------------------*/
float Motion_GetSlalomBeforeDistance( int8_t type, int8_t direction, int8_t param )
{
	if( type == TURN_0 || type == GOAL ) {
		return( 0 );
	} else {
		if(param < sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0])) {
			return init_slalom[type-1][param].before;
		} else {
			return init_slalom[type-1][sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0]) - 1].before;
		}
	}
}

/* ----------------------------------------------------------------------------------
	スラロームの後距離
-----------------------------------------------------------------------------------*/
float Motion_GetSlalomAfterDistance( int8_t type, int8_t direction, int8_t param )
{
	if( type == TURN_0 || type == GOAL ) {
		return( 0 );
	} else {
		if(param < sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0])) {
			return init_slalom[type-1][param].after;
		} else {
			return init_slalom[type-1][sizeof(init_slalom[type-1]) / sizeof(init_slalom[type-1][0]) - 1].after;
		}

	}
}

/* ---------------------------------------------------------------
	超信地旋回の開始関数
--------------------------------------------------------------- */
void Motion_StartRotate( float degree, int8_t direction )
{
	volatile float t		= 0.0f;
	volatile float angle	= DEG2RAD(degree);

	slalom_type = TURN_90;
	turn_v = init_slalom[0][0].velocity;
	before_distance = after_distance = 0.f;

	// フェールセーフ
	if( Control_GetMode() != FAULT ) {
		Control_SetMode(ROTATE);
	} else {
		return;
	}

	// 各パラメータのリセット
	Sensor_TurnOffLED();
	IMU_ResetGyroAngle_Z();
	Vehicle_ResetStraight();
	Vehicle_ResetTurning();
	Vehicle_ResetIntegral();
	Control_ResetFilterDistance();
	HAL_Delay(50);
	Control_ResetEncoderDeviation();
	Control_ResetGyroDeviation();
	Control_ResetAngleDeviation();
//	Control_ResetSensorDeviation();
//	Control_ResetFrontSensorDeviation();

	amplitude_slalom = (direction == RIGHT ? -1 : 1) * 12.0f;
	cycle_slalom = ABS(amplitude_slalom) * Mynapier_GetIntegral(TURN_90) / angle;

	// 超信地旋回
	Vehicle_ResetTimer();
	while( (Control_GetMode() != FAULT) && (t < 1.f/cycle_slalom) ) {
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

	// 各パラメータのリセット
	cycle_slalom = amplitude_slalom = 0.f;
	HAL_Delay(50);
	IMU_OffsetGyroAngle_Z();
	Vehicle_ResetStraight();
	Vehicle_ResetTurning();
	Vehicle_ResetIntegral();
	Control_ResetFilterDistance();
//	Control_ResetSensorDeviation();
//	Control_ResetFrontSensorDeviation();
	Sensor_TurnOnLED();
}

