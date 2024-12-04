#include "motion.h"

#define KT						(0.594f)			// トルク定数[mNm/A]
#define KE						(KT * 2.f*PI/60.f)	// 逆起電圧定数[mV/rpm]
#define MOT_RESIST				(5.6f)				// 巻線抵抗[Ω]
#define GEAR_RATIO				(40/7)				// ギア比

#define TORQUE_R_LOSS			(20.f)				// 右タイヤの損失トルク補償
#define TORQUE_L_LOSS			(23.f)				// 左タイヤの損失トルク補償
#define MOT_DUTY_MIN			(10)				// モータドライバのデッドタイム補償
													// 450ns * 100kHz = 0.045

volatile static float			t;
volatile static float			a, v, d;
volatile static float			alpha, omega, theta;
volatile static float 			x, y, yaw;
volatile static float			v_battery;
volatile static int16_t			duty_l, duty_r;

volatile static float			total_distance;


/* ---------------------------------------------------------------
	車両運動計算関数
--------------------------------------------------------------- */
void Vehicle_UpdateDynamics( void )
{
	volatile float			motor_rpm_l, motor_rpm_r;
	volatile float			torque_l, torque_r;

	t += SYSTEM_PERIOD;

	if( Control_GetMode() != TURN && Control_GetMode() != ROTATE ) {
		a = Motion_SetStraightAcceleration(t);
	} else;
	v += a * SYSTEM_PERIOD;
	d += v * SYSTEM_PERIOD;
	total_distance += v * SYSTEM_PERIOD;

	if( Control_GetMode() == TURN || Control_GetMode() == ROTATE ) {
		alpha = Motion_SetSlalomAngularAcceleration(t);
	} else;
	omega += alpha * SYSTEM_PERIOD;
	theta += omega * SYSTEM_PERIOD;

	yaw += omega * SYSTEM_PERIOD;
	x += v * arm_cos_f32(yaw) * SYSTEM_PERIOD;
	y += v * arm_sin_f32(yaw) * SYSTEM_PERIOD;

	// 制御量の計算
	Control_UpdateDeviation();

	v_battery = 0.99f * v_battery + 0.01f * Battery_GetVoltage();

	torque_l = TIRE/2.f * (MASS/2.f * (a + Control_GetValue_Velocity()) - INERTIA/TREAD * (alpha + Control_GetValue_Angular())) / GEAR_RATIO / 1000.f;
	torque_r = TIRE/2.f * (MASS/2.f * (a + Control_GetValue_Velocity()) + INERTIA/TREAD * (alpha + Control_GetValue_Angular())) / GEAR_RATIO / 1000.f;

	motor_rpm_l = (v - omega * TREAD) / TIRE * 60.f/PI * GEAR_RATIO;
	motor_rpm_r = (v + omega * TREAD) / TIRE * 60.f/PI * GEAR_RATIO;

	if( motor_rpm_r == 0.f ) {
		duty_r = ((MOT_RESIST * (torque_r + SIGN(torque_r) * TORQUE_R_LOSS) / KT + KE * motor_rpm_r) / v_battery);
	} else {
		duty_r = ((MOT_RESIST * (torque_r + SIGN(Encoder_GetAngle_Right()) * TORQUE_R_LOSS) / KT + KE * motor_rpm_r) / v_battery);
	}
	if( motor_rpm_l == 0.f ) {
		duty_l = ((MOT_RESIST * (torque_l + SIGN(torque_l) * TORQUE_L_LOSS) / KT + KE * motor_rpm_l) / v_battery);
	} else {
		duty_l = ((MOT_RESIST * (torque_l + SIGN(Encoder_GetAngle_Left( )) * TORQUE_L_LOSS) / KT + KE * motor_rpm_l) / v_battery);
	}

	if( Control_GetMode() == FAULT ) {
		Motor_StopPWM();
	} else {
		Motor_SetDuty_Left( duty_l + SIGN(duty_l) * MOT_DUTY_MIN);
		Motor_SetDuty_Right(duty_r + SIGN(duty_r) * MOT_DUTY_MIN);
	}
}

/* ---------------------------------------------------------------
	各車両状態変数の入力関数
--------------------------------------------------------------- */
void Vehicle_SetTimer( float value )
{
	t = value;
}

void Vehicle_SetAcceleration( float value )
{
	a = value;
}

void Vehicle_SetVelocity( float value )
{
	v = value;
}

void Vehicle_SetDistance( float value )
{
	d = value;
}

void Vehicle_SetAngularAcceleration( float value )
{
	alpha = value;
}

void Vehicle_SetGlobalX( float value )
{
	x = value;
}

void Vehicle_SetGlobalY( float value )
{
	y = value;
}

void Vehicle_SetYaw( float value )
{
	yaw = value;
}

/* ---------------------------------------------------------------
	各車両状態変数の取得関数
--------------------------------------------------------------- */
float Vehicle_GetTimer( void )
{
	return t;
}

float Vehicle_GetAcceleration( void )
{
	return a;
}

float Vehicle_GetVelocity( void )
{
	return v;
}

float Vehicle_GetDistance( void )
{
	return d;
}

float Vehicle_GetTotalDistance( void )
{
	return total_distance;
}

float Vehicle_GetAngularAcceleration( void )
{
	return alpha;
}

float Vehicle_GetAngularVelocity( void )
{
	return omega;
}

float Vehicle_GetAngle( void )
{
	return theta;
}

int16_t Vehicle_GetDuty_Right( void )
{
	return duty_r;
}

int16_t Vehicle_GetDuty_Left( void )
{
	return duty_l;
}

float Vehicle_GetVoltage( void )
{
	return v_battery;
}

float Vehicle_GetGlobalX( void )
{
	return x;
}

float Vehicle_GetGlobalY( void )
{
	return y;
}

float Vehicle_GetYaw( void )
{
	return yaw;
}

/* ---------------------------------------------------------------
	各車両状態変数の初期化関数
--------------------------------------------------------------- */
void Vehicle_ResetBatteryVoltage( void )
{
	v_battery = Battery_GetVoltage();
}

void Vehicle_ResetTimer( void )
{
	t = 0.0f;
}

void Vehicle_ResetDistance( void )
{
	d = 0.f;
}

void Vehicle_ResetTotalDistance( void )
{
	total_distance = 0.f;
}

void Vehicle_ResetAngle( void )
{
	theta = 0.f;
}

void Vehicle_ResetStraight( void )
{
	a = v = d = 0.0f;
}

void Vehicle_ResetTurning( void )
{
	alpha = omega = 0.0f;
}

void Vehicle_ResetIntegral( void )
{
	d = theta = 0.0f;
}

/* ---------------------------------------------------------------
	損失トルク調整関数
--------------------------------------------------------------- */
#define AUTORELOAD		(__HAL_TIM_GET_AUTORELOAD(&htim4))
void Vehicle_AdjustLossTorque( void )
{
	volatile float		torque_l, torque_r;
	volatile uint32_t	pulse_l, pulse_r;

	printf("Adjust Loss Torque Mode\r\n");
	while( Communicate_Receive1byte() != _ESC ) {
		// 損失トルクの入力
		scanf("%f, %f", &torque_l, &torque_r);
		if(0.f < torque_l && torque_l < 100.f && 0.f < torque_r && torque_r < 100.f) {
			v_battery = Battery_GetVoltage();
			duty_l = (int16_t)((MOT_RESIST * torque_l) / KT / v_battery) + MOT_DUTY_MIN;
			duty_r = (int16_t)((MOT_RESIST * torque_r) / KT / v_battery) + MOT_DUTY_MIN;
			pulse_l = (uint32_t)(AUTORELOAD * ABS(duty_l) / 1000);
			pulse_r = (uint32_t)(AUTORELOAD * ABS(duty_r) / 1000);

			// 入力したトルクの表示
			printf("Left : %4.1f [mN/m](%4.1f%% : %4ld), Right : %4.1f [mN/m](%4.1f%% : %4ld)\r\n",
					torque_l, (float)duty_l/10, pulse_l, torque_r, (float)duty_r/10, pulse_r);
			// 無負荷回転させる
			Motor_SetDuty_Right( duty_r );
			Motor_SetDuty_Left(  duty_l );
		} else;
	}
	duty_l = duty_r = 0;
	Motor_StopPWM();
}
