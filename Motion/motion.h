#pragma once

#include "module.h"

#define SYSTEM_PERIOD		(0.001f)  		// 積分周期[s]
#define MASS				(150.f)			// 質量[g]
#define INERTIA				(5000.f)		// 慣性モーメント[g*mm^2]
#define TIRE				(24.3f)			// タイヤの直径[mm] (小さくすると距離が延びる)
#define TREAD				(33.0f)			// 重心からタイヤまでの距離
#define START_OFFSET		(0.f)//(84.f - 40.f)	// スタートラインまでの距離 [mm]

typedef enum {	// 制御モード列挙
	FAULT	 = -1,
	NONE	 = 0,
	SEARCH	 = 1,
	FASTEST	 = 2,
	TURN	 = 3,
	ROTATE	 = 4,
	DIAGONAL = 5,
	FWALL	 = 6,
	ADJUST	 = 7,
} t_control_mode;

typedef enum {	// 走行パス列挙
	TURN_0		= 0,
	TURN_90		= 1,
	TURN_90L	= 2,
	TURN_180	= 3,
	TURN_45IN	= 4,
	TURN_135IN	= 5,
	TURN_90V	= 6,
	TURN_KOJIMA	= 7,
	TURN_45OUT	= 8,
	TURN_135OUT	= 9,
	GOAL		= 10,
} t_path_turn;

// 車両運動計算関数群(vehicle.c)
void	 	Vehicle_UpdateDynamics( void );
void 		Vehicle_AdjustLossTorque( void );
void 		Vehicle_SetTimer( float );
void 		Vehicle_SetAcceleration( float );
void 		Vehicle_SetVelocity( float );
void 		Vehicle_SetDistance( float );
void 		Vehicle_SetAngularAcceleration( float );
void 		Vehicle_SetGlobalX( float );
void 		Vehicle_SetGlobalY( float );
void 		Vehicle_SetYaw( float );
float 		Vehicle_GetTimer( void );
float 		Vehicle_GetAcceleration( void );
float 		Vehicle_GetVelocity( void );
float 		Vehicle_GetDistance( void );
float		Vehicle_GetTotalDistance( void );
float 		Vehicle_GetAngularAcceleration( void );
float 		Vehicle_GetAngularVelocity( void );
float 		Vehicle_GetAngle( void );
float 		Vehicle_GetGlobalX( void );
float		Vehicle_GetGlobalY( void );
float 		Vehicle_GetYaw( void );
int16_t		Vehicle_GetDuty_Right( void );
int16_t		Vehicle_GetDuty_Left( void );
float 		Vehicle_GetVoltage( void );
void 		Vehicle_ResetBatteryVoltage( void );
void	 	Vehicle_ResetTimer( void );
void 		Vehicle_ResetDistance( void );
void		Vehicle_ResetTotalDistance( void );
void 		Vehicle_ResetAngle( void );
void 		Vehicle_ResetStraight( void );
void 		Vehicle_ResetTurning( void );
void 		Vehicle_ResetIntegral( void );

// 車両制御関数群(control.c)
int8_t 		Control_GetMode( void );
void 		Control_SetMode( int8_t );
void 		Control_SetModeGain( int8_t );
void 		Control_UpdateDeviation( void );
void 		Control_WaitFrontWallCorrection( void );
float 		Control_GetValue_Velocity( void );
float 		Control_GetValue_Angular( void );
float 		Control_GetFilterVelocity( void );
float 		Control_GetFilterDistance( void );
void 		Control_ResetFilterDistance( void );
void 		Control_ResetEncoderDeviation( void );
void 		Control_ResetGyroDeviation( void );
void 		Control_ResetAngleDeviation( void );
void 		Control_ResetSensorDeviation( void );
void 		Control_ResetFrontSensorDeviation( void );
float 		Control_GetEncoderDeviationValue( void );
float 		Control_GetGyroDeviationValue( void );
float 		Control_GetAngleDeviationValue( void );
float 		Control_GetSensorDeviationValue( void );
float 		Control_GetFrontWallVelocityDeviationValue( void );
float 		Control_GetFrontWallAngularDeviationValue( void );

// ログ関数(log.c)
void 		Log_WriteRecodeData( void );
void 		Log_ReadRecodeData( void );
void 		Log_Store( void );
void 		Log_Load( void );

// ネイピア関数(mynapier.c)
float 		Mynapier_Calc( int8_t, float );
float 		Mynapier_GetIntegral( int8_t );

// 直進関数(straight.c)
void 		Motion_StartStraight( float, float, float, float, float );
float		Motion_GetStraightTime( float, float, float, float, float );
float 		Motion_SetStraightAcceleration( float );
void 		Motion_WaitStraight( void );

// スラローム関数群(slalom.c)
void 		Motion_StartSlalom( int8_t, int8_t, int8_t );
float 		Motion_SetSlalomAngularAcceleration( float );
void 		Motion_WaitSlalom( int8_t, int8_t, int8_t );
float 		Motion_GetSlalomTime( int8_t, int8_t );
int8_t 		Motion_GetSlalomMaxParameter( void );
float 		Motion_GetSlalomVelocity( int8_t, int8_t );
float 		Motion_GetSlalomBeforeDistance( int8_t, int8_t, int8_t );
float 		Motion_GetSlalomAfterDistance( int8_t, int8_t, int8_t );
void 		Motion_StartRotate( float, int8_t );

// 調整関連関数群(adjust.c)
void 		Adjust_RunTireDiameter( int8_t );
void 		Adjust_RunGyroSensitivity( uint8_t, int8_t );
void 		Adjust_RunPID( int8_t, int8_t );
void 		Adjust_RunSlalom( int8_t, int8_t, int8_t );
