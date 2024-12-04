#include "motion.h"

#define FAILSAFE_ENC				(30000.f)	// フェイルセーフの速度閾値
#define FAILSAFE_GYRO				(2500.f)	// フェイルセーフの角速度閾値

#define LIMIT_V_CONTROL				(25000.f)	// エンコーダの速度制限値
#define LIMIT_GYRO_CONTROL			(3000.f)	// ジャイロの角速度制限値
#define LIMIT_ROTATE_CONTROL		(800.f)		// ジャイロの角速度制限値
#define LIMIT_ANGLE_CONTROL			(1000.f)	// ジャイロの角度制限値
#define LIMIT_SENSOR_CONTROL		(800.f)		// 壁センサの角速度制限値(最短)
#define LIMIT_DIAGONAL_CONTROL		(1000.f)	// 壁センサの角速度制限値(斜め)
#define LIMIT_FWALL_V_CONTROL		(1000.f)	// 前壁制御の速度制限値
#define LIMIT_FWALL_OMEGA_CONTROL	(800.f)		// 前壁制御の角速度制限値

// 制御パラメータ用構造体
typedef struct {
	float kp;	// 比例制御量
	float ki;	// 積分制御量
	float kd;	// 微分制御量
} t_control_gain;

// 各制御偏差用構造体
typedef struct {
	volatile float	deviation;
	volatile float	dif_deviation;
	volatile float	intg_deviation;
	volatile float	control_value;
	volatile float	error;
} t_control_deviation;

// 各制御のPIDゲイン												P	  I		 D
volatile const t_control_gain	gain_search_enc		= {   20.f,  1.f,   0.f		};	// エンコーダの速度ゲイン
volatile const t_control_gain	gain_search_gyro 	= {   50.f,  5.f,   0.f		};	// ジャイロの角速度ゲイン
volatile const t_control_gain	gain_search_angle 	= { 5000.f,  0.f,   0.f  	};	// ジャイロの角度ゲイン
volatile const t_control_gain	gain_search_sensor	= { 1000.f,  0.f, 100.f  	};	// 壁制御の角速度ゲイン

volatile const t_control_gain	gain_fastest_enc	= {   30.f,  1.5f,  0.f		};	// エンコーダの速度ゲイン
volatile const t_control_gain	gain_fastest_gyro 	= {  300.f, 10.f,   0.f		};	// ジャイロの角速度ゲイン
volatile const t_control_gain	gain_fastest_angle	= {15000.f,  0.f,   0.f  	};	// ジャイロの角度ゲイン
volatile const t_control_gain	gain_fastest_sensor	= { 3000.f,  0.f, 250.f  	};	// 壁制御の角速度ゲイン

volatile const t_control_gain	gain_diagonal	 	= { 3000.f,  0.f, 250.f  	};	// 斜め壁制御の角速度ゲイン
volatile const t_control_gain	gain_rotate_gyro	= {   50.f,  5.f,	0.f		};	// ジャイロの角速度ゲイン
volatile const t_control_gain	gain_rotate_angle	= { 5000.f,  0.f,   0.f  	};	// ジャイロの角度ゲイン
volatile const t_control_gain	gain_fwall_v	 	= {  500.f,  2.f,   2.f   	};	// 前壁制御の速度ゲイン
volatile const t_control_gain	gain_fwall_omega 	= { 5000.f, 40.f,   2.f   	};	// 前壁制御の角速度ゲイン

// ローカル関数群
void 	Control_UpdateFilterVelocity( void );
void 	Control_UpdateEncoderDeviation( void );
void 	Control_UpdateGyroDeviation( void );
void 	Control_UpdateAngleDeviation( void );
void	Control_UpdateSensorDeviation( void );
void 	Control_UpdateFrontSensorDeviation( void );
void 	Control_FailSafe( void );

volatile static t_control_gain			gain_enc;
volatile static t_control_gain			gain_gyro;
volatile static t_control_gain			gain_angle;
volatile static t_control_gain 			gain_straight;

volatile static t_control_deviation		enc_v;
volatile static t_control_deviation		gyro;
volatile static t_control_deviation		angle;
volatile static t_control_deviation		sensor;
volatile static t_control_deviation 	fwall_v;
volatile static t_control_deviation 	fwall_omega;

// 制御モード変数
volatile static int8_t		control_mode = NONE;

// 速度フィルター変数群
volatile static float		filter_v			= 0.f;
volatile static float		filter_d			= 0.f;

/* ---------------------------------------------------------------
	車両制御モードを取得する関数
--------------------------------------------------------------- */
int8_t Control_GetMode( void )
{
	return control_mode;
}

/* ---------------------------------------------------------------
	車両制御モードを入力する関数
--------------------------------------------------------------- */
void Control_SetMode( int8_t mode )
{
	if( control_mode != FAULT || (mode == NONE) ) {
		if( control_mode == FAULT ) {
			HAL_Delay( 200 );
		} else;
		control_mode = mode;
	} else;
}

/* ---------------------------------------------------------------
	車両制御モードを入力する関数
--------------------------------------------------------------- */
void Control_SetModeGain( int8_t mode )
{
	if( mode == SEARCH ) {
		gain_enc 		= gain_search_enc;
		gain_gyro 		= gain_search_gyro;
		gain_angle 		= gain_search_angle;
		gain_straight 	= gain_search_sensor;
	} else {
		gain_enc 		= gain_fastest_enc;
		gain_gyro 		= gain_fastest_gyro;
		gain_angle 		= gain_fastest_angle;
		gain_straight	= gain_fastest_sensor;
	}
}

/* ---------------------------------------------------------------
	各制御量の更新関数
--------------------------------------------------------------- */
void Control_UpdateDeviation( void )
{
	Control_UpdateEncoderDeviation();
	Control_UpdateGyroDeviation();
	Control_UpdateAngleDeviation();
//	Control_UpdateSensorDeviation();
//	Control_UpdateFrontSensorDeviation();
	Control_FailSafe();
}

/* ---------------------------------------------------------------
	速度制御量の取得関数
--------------------------------------------------------------- */
float Control_GetValue_Velocity( void )
{
	if( control_mode == FWALL ) {
		return fwall_v.control_value;
	} else {
		return enc_v.control_value;
	}
}

/* ---------------------------------------------------------------
	角速度制御量の取得関数
--------------------------------------------------------------- */
float Control_GetValue_Angular( void )
{
	if( control_mode == FWALL ) {
		return fwall_omega.control_value;
	} else if( control_mode == SEARCH || control_mode == FASTEST || control_mode == DIAGONAL ) {
		return (gyro.control_value + angle.control_value + sensor.control_value);
	} else if( control_mode == ROTATE ){
		return gyro.control_value + angle.control_value;
	} else {
		return (gyro.control_value + angle.control_value);
	}
}

/* ---------------------------------------------------------------
	速度フィルタ計算関数
--------------------------------------------------------------- */
void Control_UpdateFilterVelocity( void )
{
	float alpha = 0.85f;
	filter_v = alpha * (filter_v + IMU_GetAccel_X() * SYSTEM_PERIOD)
			 + (1 - alpha) * TIRE/4.f * (Encoder_GetAngle_Right() + Encoder_GetAngle_Left()) / SYSTEM_PERIOD;
//	filter_v = filter_v + IMU_GetAccel_X() * SYSTEM_PERIOD;
	filter_d += filter_v * SYSTEM_PERIOD;

	// エンコーダの値のリセット
	Encoder_ResetCount_Right();
	Encoder_ResetCount_Left();
}

/* ---------------------------------------------------------------
	フィルタした速度の取得関数
--------------------------------------------------------------- */
float Control_GetFilterVelocity( void )
{
	return filter_v;
}

float Control_GetFilterDistance( void )
{
	return filter_d;
}

void Control_ResetFilterDistance( void )
{
	filter_d = 0.f;
}

/* ---------------------------------------------------------------
	エンコーダによる速度制御量の計算関数
--------------------------------------------------------------- */
void Control_UpdateEncoderDeviation( void )
{
	static volatile float	val_control = 0.f;

	Control_UpdateFilterVelocity();
	enc_v.deviation = Vehicle_GetVelocity() - filter_v;
	enc_v.dif_deviation -= enc_v.control_value;
	enc_v.intg_deviation += (enc_v.deviation - (val_control - enc_v.control_value) / gain_enc.kp);
	enc_v.error += enc_v.deviation;
	val_control = (gain_enc.kp * enc_v.deviation) + (gain_enc.ki * enc_v.intg_deviation) + (gain_enc.kd * enc_v.dif_deviation);
	enc_v.control_value = SIGN(val_control) * MIN( LIMIT_V_CONTROL, ABS(val_control) );
}

/* ---------------------------------------------------------------
	エンコーダによる速度制御量の初期化関数
--------------------------------------------------------------- */
void Control_ResetEncoderDeviation( void )
{
	enc_v.deviation = enc_v.dif_deviation = enc_v.intg_deviation = enc_v.control_value = 0.0f;
	enc_v.error = 0.f;
}

/* ---------------------------------------------------------------
	ジャイロによる角速度制御量の計算関数
--------------------------------------------------------------- */
void Control_UpdateGyroDeviation( void )
{
	static volatile float	val_control = 0.f;

	// ジャイロ角速度の偏差計算
	if( control_mode == SEARCH || control_mode == FASTEST || control_mode == DIAGONAL ) {
		gyro.intg_deviation = gyro.dif_deviation = 0.f;
		if(control_mode == FASTEST) {
/*			if(Wall_GetDeviation(LEFT) != 0.f || Wall_GetDeviation(RIGHT) != 0.f) {
				gyro.deviation = 0.f;
			} else {
*/				gyro.deviation = Vehicle_GetAngularVelocity() - IMU_GetGyro_Z();
//			}
		} else if(control_mode == DIAGONAL) {
			gyro.deviation = Vehicle_GetAngularVelocity() - IMU_GetGyro_Z();
		} else {
			gyro.intg_deviation = 0.f;
		}
		val_control = (gain_gyro.kp * gyro.deviation) + (gain_gyro.ki * gyro.intg_deviation) + (gain_gyro.kd * gyro.dif_deviation);
		gyro.control_value = SIGN(val_control) * MIN(LIMIT_GYRO_CONTROL, ABS(val_control));
	} else if( control_mode == TURN || control_mode == ADJUST ) {
		gyro.deviation = Vehicle_GetAngularVelocity() - IMU_GetGyro_Z();
//		gyro.intg_deviation = gyro.dif_deviation = 0.f;
		gyro.dif_deviation -= gyro.control_value;
		gyro.intg_deviation += (gyro.deviation - (val_control - gyro.control_value) / gain_gyro.kp);
		val_control = (gain_gyro.kp * gyro.deviation) + (gain_gyro.ki * gyro.intg_deviation) + (gain_gyro.kd * gyro.dif_deviation);
		gyro.control_value = SIGN(val_control) * MIN(LIMIT_GYRO_CONTROL, ABS(val_control));
	} else if( control_mode == ROTATE ) {
		gyro.deviation = Vehicle_GetAngularVelocity() - IMU_GetGyro_Z();
		gyro.dif_deviation -= gyro.control_value;
		gyro.intg_deviation += (gyro.deviation - (val_control - gyro.control_value) / gain_rotate_gyro.kp);
		val_control = (gain_rotate_gyro.kp * gyro.deviation) + (gain_rotate_gyro.ki * gyro.intg_deviation) + (gain_rotate_gyro.kd * gyro.dif_deviation);
		gyro.control_value = SIGN(val_control) * MIN(LIMIT_ROTATE_CONTROL, ABS(val_control));
	} else {
		gyro.dif_deviation = gyro.intg_deviation = 0.f;
		gyro.control_value = 0.f;
	}
	gyro.error += gyro.deviation;
}

/* ---------------------------------------------------------------
	ジャイロによる角速度制御量の初期化関数
--------------------------------------------------------------- */
void Control_ResetGyroDeviation( void )
{
	gyro.deviation = gyro.dif_deviation = gyro.intg_deviation = gyro.control_value = 0.0f;
	gyro.error = 0.f;
}

/* ---------------------------------------------------------------
	ジャイロによる角度制御量の計算関数
--------------------------------------------------------------- */
void Control_UpdateAngleDeviation( void )
{
	static volatile float	val_control = 0.f;

	if( control_mode == SEARCH || control_mode == FASTEST || control_mode == DIAGONAL ) {
		angle.deviation = angle.dif_deviation = angle.intg_deviation = 0.f;
		val_control = (gain_angle.kp * angle.deviation) + (gain_angle.ki * angle.intg_deviation) + (gain_angle.kd * angle.dif_deviation);
		angle.control_value = SIGN(val_control) * MIN(LIMIT_ANGLE_CONTROL, ABS(val_control));
	} else if( control_mode == TURN || control_mode == ADJUST ) {
		angle.deviation = Vehicle_GetAngle() - IMU_GetGyroAngle_Z();
		angle.dif_deviation -= angle.deviation;
		angle.intg_deviation += (angle.deviation - (val_control - angle.control_value) / gain_angle.kp);
		val_control = (gain_angle.kp * angle.deviation) + (gain_angle.ki * angle.intg_deviation) + (gain_angle.kd * angle.dif_deviation);
		angle.control_value = SIGN(val_control) * MIN(LIMIT_ANGLE_CONTROL, ABS(val_control));
	} else if( control_mode == ROTATE ) {
		angle.deviation = Vehicle_GetAngle() - IMU_GetGyroAngle_Z();
		angle.dif_deviation -= angle.deviation;
		angle.intg_deviation += (angle.deviation - (val_control - angle.control_value) / gain_rotate_gyro.kp);
		val_control = (gain_rotate_angle.kp * angle.deviation) + (gain_rotate_angle.ki * angle.intg_deviation) + (gain_rotate_angle.kd * angle.dif_deviation);
		angle.control_value = SIGN(val_control) * MIN(LIMIT_ANGLE_CONTROL, ABS(val_control));
	} else {
		angle.deviation = angle.dif_deviation = angle.intg_deviation = 0.f;
		angle.control_value = 0.f;
	}
}

/* ---------------------------------------------------------------
	ジャイロによる角度制御量の初期化関数
--------------------------------------------------------------- */
void Control_ResetAngleDeviation( void )
{
	angle.deviation = angle.dif_deviation = angle.intg_deviation = angle.control_value = 0.0f;
	angle.error = 0.f;
}

#if 0
/* ---------------------------------------------------------------
	壁センサによる制御量の計算関数
--------------------------------------------------------------- */
void Control_UpdateSensorDeviation( void )
{
	static volatile float	val_control = 0.f;

	// 探索・最短時の直線壁制御
	if( (control_mode == SEARCH) || (control_mode == FASTEST) ) {
		// 両壁ありの場合
		if( (Wall_GetDeviation(LEFT) != 0.f) && (Wall_GetDeviation(RIGHT) != 0.f) ) {
			sensor.deviation = Wall_GetDeviation(LEFT) - Wall_GetDeviation(RIGHT);
		// 左壁のみありの場合
		} else if( Wall_GetDeviation(LEFT) != 0.f ) {
			sensor.deviation = 2 * Wall_GetDeviation(LEFT);
		// 右壁のみありの場合
		} else if( Wall_GetDeviation(RIGHT) != 0.f ) {
			sensor.deviation = -2 * Wall_GetDeviation(RIGHT);
		// 両壁共なしの場合
		} else {
			sensor.deviation = 2 * (SENSOR_LENGTH * (Vehicle_GetAngle() - IMU_GetGyroAngle_Z()) - Vehicle_GetGap());
		}
		sensor.dif_deviation -= sensor.deviation;
		sensor.intg_deviation += (sensor.deviation - (val_control - sensor.control_value) / gain_straight.kp);
		sensor.error += sensor.deviation;
		val_control = -(gain_straight.kd + Vehicle_GetVelocity() / SENSOR_LENGTH) * IMU_GetGyro_Z() + gain_straight.kp / SENSOR_LENGTH * sensor.deviation;

		if( Vehicle_GetVelocity() < 50.f ) {
			Vehicle_ResetTurning();
			sensor.deviation = sensor.intg_deviation = sensor.dif_deviation = 0;
			val_control = 0.f;
		} else;

		if( control_mode == SEARCH ) {
			sensor.control_value = SIGN(val_control) * MIN( LIMIT_SEARCH_CONTROL, ABS(val_control) );
		} else {
			sensor.control_value = SIGN(val_control) * MIN( LIMIT_SENSOR_CONTROL, ABS(val_control) );
		}

	// 最短時の斜め壁制御
	} else if( control_mode == DIAGONAL ) {
		// 左柱のみありの場合
		if( Wall_GetDeviation(FRONT+LEFT) < 0.f ) {
			sensor.deviation = 2 * Wall_GetDeviation(FRONT+LEFT);
		// 右柱のみありの場合
		} else if( Wall_GetDeviation(FRONT+RIGHT) < 0.f ) {
			sensor.deviation = -2 * Wall_GetDeviation(FRONT+RIGHT);
		} else {
			sensor.deviation = 2 * (SENSOR_LENGTH * (Vehicle_GetAngle() - IMU_GetGyroAngle_Z()) - Vehicle_GetGap());
		}
		sensor.dif_deviation -= sensor.deviation;
		sensor.intg_deviation += (sensor.deviation - (val_control - sensor.control_value) / gain_diagonal.kp);
		sensor.error += sensor.deviation;
		val_control = -(gain_diagonal.kd + filter_v / SENSOR_LENGTH) * IMU_GetGyro_Z() + gain_diagonal.kp / SENSOR_LENGTH * sensor.deviation;

		if( Vehicle_GetVelocity() < 50.f ) {
			Vehicle_ResetTurning();
			sensor.deviation = sensor.intg_deviation = sensor.dif_deviation = 0;
			val_control = 0.f;
		} else;
		sensor.control_value = SIGN(val_control) * MIN( LIMIT_DIAGONAL_CONTROL, ABS(val_control) );

	// ターン時の壁制御
	} else {
		val_control = 0.0f;
		Control_ResetSensorDeviation();
	}
}

/* ---------------------------------------------------------------
	壁センサによる制御量の初期化関数
--------------------------------------------------------------- */
void Control_ResetSensorDeviation( void )
{
	sensor.deviation = sensor.dif_deviation = sensor.intg_deviation = sensor.control_value = 0.0f;
	sensor.error = 0.f;
}

/* ---------------------------------------------------------------
	前壁センサによる制御量の計算関数
--------------------------------------------------------------- */
void Control_UpdateFrontSensorDeviation( void )
{
	volatile static float val_control_v 	= 0.f;
	volatile static float val_control_omega = 0.f;

	if( control_mode == FWALL ) {
		// 前壁があるとき
		if( (Wall_GetDeviation(FRONT+LEFT) != 0) && (Wall_GetDeviation(FRONT+RIGHT) != 0) ) {
			Control_ResetEncoderDeviation();
			Control_ResetGyroDeviation();
			Control_ResetAngleDeviation();
			Control_ResetSensorDeviation();

			// 速度制御
			fwall_v.deviation = (Wall_GetDeviation(FRONT+LEFT) + Wall_GetDeviation(FRONT+RIGHT)) / 2;
			fwall_v.dif_deviation -= fwall_v.deviation;
			fwall_v.intg_deviation += (fwall_v.deviation - (val_control_v - fwall_v.control_value) / gain_fwall_v.kp);
			val_control_v = gain_fwall_v.kp * fwall_v.deviation + gain_fwall_v.ki * fwall_v.intg_deviation + gain_fwall_v.kd * fwall_v.dif_deviation;
			fwall_v.control_value = SIGN(val_control_v) * MIN(LIMIT_FWALL_V_CONTROL, ABS(val_control_v));

			// 角速度制御
			fwall_omega.deviation = (Wall_GetDeviation(FRONT+RIGHT) - Wall_GetDeviation(FRONT+LEFT)) / (4.f * TREAD);
			fwall_omega.dif_deviation -= fwall_omega.deviation;
			fwall_omega.intg_deviation += (fwall_omega.deviation - (val_control_omega - fwall_omega.control_value) / gain_fwall_omega.kp);
			val_control_omega = gain_fwall_omega.kp * fwall_omega.deviation + gain_fwall_omega.ki * fwall_omega.intg_deviation + gain_fwall_omega.kd * fwall_omega.dif_deviation;
			fwall_omega.control_value = SIGN(val_control_omega) * MIN(LIMIT_FWALL_OMEGA_CONTROL, ABS(val_control_omega));

		// 前壁がないとき
		} else {
			val_control_v = val_control_omega = 0.f;
			Control_ResetFrontSensorDeviation();
		}
	} else {
		val_control_v = val_control_omega = 0.f;
		Control_ResetFrontSensorDeviation();
	}
}

/* ---------------------------------------------------------------
	前壁制御が安定するまで待機する関数
--------------------------------------------------------------- */
void Control_WaitFrontWallCorrection( void )
{
	uint32_t tick = Interrupt_GetGlobalTime();
	uint32_t timer = Interrupt_GetGlobalTime();

	Control_SetMode(FWALL);
	// 1秒経過または前壁制御が一定秒数間安定するまで待機
	while( Interrupt_GetGlobalTime() - tick < 1000 && Interrupt_GetGlobalTime() - timer < 200 ) {
		if( control_mode == FAULT ) {
			break;
		} else;

		if( fwall_v.control_value > 300.f || fwall_omega.control_value > 10.f ) {
			timer = Interrupt_GetGlobalTime();
		} else;
	}
}

/* ---------------------------------------------------------------
	前壁センサによる制御量の初期化関数
--------------------------------------------------------------- */
void Control_ResetFrontSensorDeviation( void )
{
	fwall_v.deviation = fwall_v.intg_deviation = fwall_v.dif_deviation = fwall_v.control_value = 0.0f;
	fwall_omega.deviation = fwall_omega.intg_deviation = fwall_omega.dif_deviation = fwall_omega.control_value = 0.0f;
}
#endif

/* ---------------------------------------------------------------
	フェイルセーフ関数
--------------------------------------------------------------- */
void Control_FailSafe( void )
{
	if( ABS(enc_v.error) > FAILSAFE_ENC ) {
		control_mode = FAULT;
	} else;

	if( ABS(gyro.error) > FAILSAFE_GYRO ) {
		control_mode = FAULT;
	} else;
}

/* ---------------------------------------------------------------
	各センサにおける制御量を取得する関数
--------------------------------------------------------------- */
float Control_GetEncoderDeviationValue( void )
{
	return enc_v.control_value;
}

float Control_GetGyroDeviationValue( void )
{
	return gyro.control_value;
}

float Control_GetAngleDeviationValue( void )
{
	return angle.control_value;
}

float Control_GetSensorDeviationValue( void )
{
	return sensor.control_value;
}

float Control_GetFrontWallVelocityDeviationValue( void )
{
	return fwall_v.control_value;
}

float Control_GetFrontWallAngularDeviationValue( void )
{
	return fwall_omega.control_value;
}
