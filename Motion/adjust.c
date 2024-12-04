#include "motion.h"

/* ---------------------------------------------------------------
	タイヤ直径の調整関数
--------------------------------------------------------------- */
void Adjust_RunTireDiameter( int8_t section )
{
	Control_SetModeGain(SEARCH);
//	Sensor_SetFrequency(SEARCH);
	Sensor_TurnOnLED();

	Control_SetMode( FASTEST );
	Motion_StartStraight( 4000.f, 4000.f, 350.f, 0.f, 90.f*section + START_OFFSET );
	Motion_WaitStraight();
	HAL_Delay( 200 );
}

/* ---------------------------------------------------------------
	ジャイロの角度係数の調整関数
--------------------------------------------------------------- */
void Adjust_RunGyroSensitivity( uint8_t count, int8_t direction )
{
	Control_SetModeGain(SEARCH);
	Motion_StartRotate(360.f*(float)count, direction);
	HAL_Delay( 200 );
}

/* ---------------------------------------------------------------
	PIDゲインの調整関数
--------------------------------------------------------------- */
void Adjust_RunPID( int8_t mode, int8_t param )
{
/*	t_init_straight temp = Route_GetParameters(mode, param);
	const float acceleration = temp.acceleration;
	const float deceleration = temp.deceleration;
	const float max_velocity = temp.max_velocity;
*/
	const float acceleration = 4000.f;
	const float deceleration = 4000.f;
	const float max_velocity = 1000.f;

	float 		turn_velocity = 0.f;
	float 		after_distance = 0.f;

	Control_SetModeGain(SEARCH);
//	Control_SetModeGain(FASTEST);
//	Sensor_SetFrequency(FASTEST);
	Sensor_TurnOnLED();

	// 吸引ファンの起動
	if( param > 0 ) {
		SuctionFan_Start(0.6f);
		HAL_Delay( 200 );
	} else;

	if( mode == FASTEST ) {
		turn_velocity  = Motion_GetSlalomVelocity(TURN_90L, param);
		after_distance = Motion_GetSlalomAfterDistance(TURN_90L, LEFT, param);

		Control_SetMode( FASTEST );
		Motion_StartStraight( acceleration, deceleration, turn_velocity, turn_velocity, 180.f - 15.f + START_OFFSET );
		Motion_WaitStraight();

		Motion_StartSlalom( TURN_90L, LEFT, param );
		Motion_WaitSlalom( TURN_90L, LEFT, param );

		Control_SetMode( FASTEST );
		Motion_StartStraight( acceleration, deceleration, max_velocity, 0.f, after_distance + 180.f*2.f + 11.f );
		Motion_WaitStraight();

	} else if( mode == DIAGONAL ) {
		turn_velocity  = Motion_GetSlalomVelocity(TURN_135IN, param);
		after_distance = Motion_GetSlalomAfterDistance(TURN_135IN, LEFT, param);

		Control_SetMode( ADJUST );
		Motion_StartStraight( acceleration, deceleration, turn_velocity, turn_velocity, 90.f - 15.f + START_OFFSET );
		Motion_WaitStraight();

		Motion_StartSlalom( TURN_135IN, RIGHT, param );
		Motion_WaitSlalom( TURN_135IN, RIGHT, param );
		turn_velocity  = Motion_GetSlalomVelocity(TURN_45OUT, param);

		Control_SetMode( DIAGONAL );
		Motion_StartStraight( acceleration, deceleration, max_velocity, turn_velocity, after_distance + 45.f*SQRT2*21.f - 15.f );
		Motion_WaitStraight();

		Motion_StartSlalom( TURN_45OUT, RIGHT, param );
		Motion_WaitSlalom( TURN_45OUT, RIGHT, param );

		Motion_StartSlalom( TURN_135IN, RIGHT, param );
		Motion_WaitSlalom( TURN_135IN, RIGHT, param );

		Control_SetMode( DIAGONAL );
		Motion_StartStraight( acceleration, deceleration, max_velocity, 0.f, after_distance + 45.f*SQRT2*23.f + 11.f );
		Motion_WaitStraight();

	} else;

	SuctionFan_Stop();
	HAL_Delay( 200 );
}

/* ---------------------------------------------------------------
	スラロームの調整関数
--------------------------------------------------------------- */
/*void Adjust_RunSlalom( int8_t type, int8_t direction, int8_t param )
{
	float	acceleration;
	float 	turn_velocity;
	float	after_distance;
	int8_t 	turn_param;

	t_init_straight temp = Route_GetParameters(FASTEST, param);
	acceleration = temp.acceleration;
	turn_velocity = Motion_GetSlalomVelocity( type, param );
	after_distance = Motion_GetSlalomAfterDistance( type, direction, param );
	if(param == 0) {
		turn_param = 0;
	} else {
		 turn_param = 2 * (param - 1) + 5;
	}

	Control_SetMode( ADJUST );
	if( type == TURN_90 || param == 0 ) {
		Control_SetModeGain(SEARCH);
//		Sensor_SetFrequency(SEARCH);
	} else {
		Control_SetModeGain(FASTEST);
//		Sensor_SetFrequency(FASTEST);
		SuctionFan_Start(0.6f);
		HAL_Delay(200);
	}
	Sensor_TurnOnLED();

	// 探索スラローム
	if( type == TURN_90 ) {
		Motion_StartStraight( 4000.f, 4000.f, turn_velocity, turn_velocity, 45.f + START_OFFSET );
		Motion_WaitStraight();
		Motion_StartSlalom( type, direction, turn_param );
		Motion_WaitSlalom( TURN_0, direction, turn_param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( 4000.f, 4000.f, turn_velocity, 0.f, 45.f );

	// 大回り90度スラローム＆180度スラローム
	} else if( type == TURN_90L || type == TURN_180  ) {
		Motion_StartStraight( acceleration, acceleration, turn_velocity, turn_velocity, 90.f + START_OFFSET - 20.f );
		Motion_WaitStraight();
		Motion_SetSlalomAcceleration(acceleration);
		Motion_StartSlalom( type, direction, turn_param );
		Motion_WaitSlalom( type, direction, turn_param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( acceleration, acceleration, turn_velocity, 0.f, 90.f + after_distance );

	// 斜め侵入スラローム
	} else if( type == TURN_45IN || type == TURN_135IN ) {
		Motion_StartStraight( acceleration, acceleration, turn_velocity, turn_velocity, 90.f + START_OFFSET - 20.f );
		Motion_WaitStraight();
		Motion_SetSlalomAcceleration(acceleration);
		Motion_StartSlalom( type, direction, turn_param );
		Motion_WaitSlalom( type, direction, turn_param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( acceleration, acceleration, turn_velocity, 0.f, 90.f*SQRT2 + after_distance );

	// 斜め脱出スラローム
	} else if( type == TURN_45OUT || type == TURN_135OUT ) {
		Motion_StartStraight( acceleration, acceleration, turn_velocity, turn_velocity, 90.f*SQRT2 - 20.f );
		Motion_WaitStraight();
		Motion_SetSlalomAcceleration(acceleration);
		Motion_StartSlalom( type, direction, turn_param );
		Motion_WaitSlalom( type, direction, turn_param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( acceleration, acceleration, turn_velocity, 0.f, 90.f + after_distance );

	// 斜め90度スラローム
	} else if( type == TURN_90V || type == TURN_KOJIMA ) {
		Motion_StartStraight( acceleration, acceleration, turn_velocity, turn_velocity, 90.f*SQRT2 - 20.f );
		Motion_WaitStraight();
		Motion_SetSlalomAcceleration(acceleration);
		Motion_StartSlalom( type, direction, turn_param );
		Motion_WaitSlalom( type, direction, turn_param );
		Control_SetMode( ADJUST );
		Motion_StartStraight( acceleration, acceleration, turn_velocity, 0.f, 90.f*SQRT2 + after_distance );

	} else;

	Motion_WaitStraight();
	HAL_Delay(200);
	SuctionFan_Stop();
}*/
