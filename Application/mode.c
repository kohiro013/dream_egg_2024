#include "module.h"
#include "motion.h"
#include "application.h"

// ホイールクリック設定
#define ONE_CLICK_ANGLE			(PI/6.f)
#define ONE_CLICK_GAIN			(150)
#define ONE_CLICK_DUTY_LIMIT	(150)

static void 	selectWheelCrickMode(int8_t*, int8_t, int8_t);
static void		resetAllParams(void);
static int8_t	resetStartPreparation(void);

/* ---------------------------------------------------------------
	探索モード
--------------------------------------------------------------- */
void mode_search( int8_t param ) {
	int8_t 	sw_side = resetStartPreparation();
	switch( param ) {
		case 1:
//			Search_RunStart( ADACHI, sw_side == RIGHT );
			SuctionFan_Start(0.5f);
			while(Control_GetMode() != FAULT && !Switch_GetIsPush()) {
				if( Vehicle_GetVoltage() < Battery_GetLimitVoltage() ) {
					Control_SetMode( FAULT );
				} else {}
			}
			SuctionFan_Stop();
		break;

		case 2:
			SuctionFan_Start(0.4f);
			while(Control_GetMode() != FAULT && !Switch_GetIsPush()) {
				if( Vehicle_GetVoltage() < Battery_GetLimitVoltage() ) {
					Control_SetMode( FAULT );
				} else {}
			}
			SuctionFan_Stop();
		break;

		case 3:
			SuctionFan_Start(0.3f);
			while(Control_GetMode() != FAULT && !Switch_GetIsPush()) {
				if( Vehicle_GetVoltage() < Battery_GetLimitVoltage() ) {
					Control_SetMode( FAULT );
				} else {}
			}
			SuctionFan_Stop();
/*			if(sw_side == FRONT) {
				HAL_Delay(500);
				IMU_ResetReference();
				resetAllParams();
			} else;
			Adjust_RunGyroSensitivity(1, RIGHT);
*/		break;

		case 4:
/*			if(sw_side == FRONT) {
				HAL_Delay(500);
				IMU_ResetReference();
				resetAllParams();
			} else;
			Adjust_RunPID(FASTEST, 0);
*/		break;

		case 5:
/*			if(sw_side == FRONT) {
				HAL_Delay(500);
				IMU_ResetReference();
				resetAllParams();
			} else;
			LED_LightBinary(0xff);
			Control_SetModeGain(SEARCH);
			Sensor_TurnOnLED();
//			Sensor_SetFrequency(SEARCH);
			Control_SetMode(FWALL);
			while(Control_GetMode() != FAULT && !Switch_GetIsPush()) {
				if( Vehicle_GetVoltage() < Battery_GetLimitVoltage() ) {
					Control_SetMode( FAULT );
				} else {}
			}
*/		break;

		case 6:
			CleanerFan_Start(0.5f);
			while(Control_GetMode() != FAULT && !Switch_GetIsPush()) {
				if( Vehicle_GetVoltage() < Battery_GetLimitVoltage() ) {
					Control_SetMode( FAULT );
				} else {}
			}
			CleanerFan_Stop();
		break;

		case 7:
			if(sw_side == FRONT) {
				HAL_Delay(500);
				IMU_ResetReference();
				resetAllParams();
			} else;
			LED_LightBinary(0xff);
			Control_SetModeGain(SEARCH);
			Control_SetMode(TURN);
			while(Control_GetMode() != FAULT && !Switch_GetIsPush()) {
				if( Vehicle_GetVoltage() < Battery_GetLimitVoltage() ) {
					Control_SetMode( FAULT );
				} else {}
			}
		break;
	}
}

/* ---------------------------------------------------------------
	最短モード
--------------------------------------------------------------- */
void mode_fastest( int8_t param ) {
	int8_t 	sw_side = resetStartPreparation();
	if(sw_side == FRONT) {
		HAL_Delay(500);
		IMU_ResetReference();
		resetAllParams();
	} else;
	Route_DebugData();
	Cleaner_Run(false);
}

/* ---------------------------------------------------------------
	調整モード
--------------------------------------------------------------- */
void mode_adjust( int8_t param ) {
	resetStartPreparation();
	printf("Adjust Mode!!\r\n");
}

/* ---------------------------------------------------------------
	オートモード
--------------------------------------------------------------- */
void mode_auto( int8_t param ) {
	resetStartPreparation();
	printf("Auto Mode!!\r\n");
}

/* ---------------------------------------------------------------
	デバッグモード
--------------------------------------------------------------- */
void mode_cleaner( int8_t param ) {
	printf("Cleaner Mode!!\r\n");
	switch( param ) {
		case 1:
			printf("param: start\r\n");
//			HAL_Delay(200);
			IMU_ResetReference();
			resetAllParams();
			LED_LightBinary(0x00);
			Cleaner_Run( true );
		break;

		case 2:
			printf("param: stop\r\n");
		break;

		case 3:
			printf("param: return\r\n");
		break;

		case 4:
			printf("param: rc mode\r\n");
		break;

		case 5:
			printf("param: path\r\n");
			uint8_t data[1024];
			uint16_t length;

			Telemetry_GetData(data, &length);
			Route_SetPath(data, length);
			Path_DisplayAll( false );
		break;
	}
}

/* ---------------------------------------------------------------
	デバッグモード
--------------------------------------------------------------- */
void mode_debug( int8_t param ) {
	resetStartPreparation();
	printf("Debug Mode!!\r\n");
}

/* ---------------------------------------------------------------
	モードセレクト
--------------------------------------------------------------- */
void Mode_SelectRunMode( void )
{
	static int8_t	run_mode		= 0;
	static int8_t	telem_mode		= 0;
	static int8_t	param			= 0;
	int8_t			select_mode		= false;
	int8_t			is_switch		= false;

	void (*p_function[])(int8_t) = {
			mode_auto, mode_search, mode_fastest, mode_adjust, mode_debug, mode_cleaner
	};

	// スイッチが押下されるまでモード選択
	Control_SetMode(NONE);
	resetAllParams();
	Vehicle_ResetBatteryVoltage();
	LED_LightBinary(0x00);

	while( 1 ) {
		is_switch = Switch_GetIsPush();

		// 長押しでデバッグモード
		if(is_switch == 2) {
			select_mode = false;
			LED_LightBinary( 0x00 );
			if( select_mode == false ) {
				run_mode = -1;
				break;
			} else {
				select_mode = false;
			}
			while( is_switch == 2 );
		// スイッチ押下でモードとパラメータを決定
		} else if(is_switch == 1) {
			if( select_mode == false ) {
				select_mode = true;
				HAL_Delay(10);
			} else {
				break;
			}
		} else;

		if( select_mode == false ) {
			selectWheelCrickMode(&run_mode, 4, RIGHT);
			LED_LightBinary(1 << run_mode);
		} else {
			selectWheelCrickMode(&param, 7, RIGHT);
			LED_LightBinary(0x00);
			LED_TimerBinary(param + 1, 50);
		}

		Myshell_Execute();
		telem_mode = Telemetry_Decode();
		if( telem_mode ) {
			run_mode = 4;
			param = telem_mode - 1;
			break;
		} else {}
	}
	LED_TimerBinary(param + 1, 200);

	resetAllParams();
	(*p_function[run_mode+1])(param+1);
	resetAllParams();

	LED_LightBinary(0x00);
	if(Control_GetMode() != NONE) {
		Log_Store();
	} else {
		HAL_Delay(100);
	}

	if(Control_GetMode() == FAULT) {
		while(Switch_GetIsPush() == false) {
			LED_LightBinary(0x55);	HAL_Delay(100);
			LED_LightBinary(0xaa);	HAL_Delay(100);
		}
	} else;
}

/* ---------------------------------------------------------------
	ホイールクリック関数
--------------------------------------------------------------- */
static void selectWheelCrickMode( int8_t* mode, int8_t num_mode, int8_t direction )
{
	float 			enc_angle	= 0.0f;
	int16_t 		duty_click	= 0;

	if(direction == RIGHT)		enc_angle = Encoder_GetAngle_Right();
	else if(direction == LEFT)	enc_angle = Encoder_GetAngle_Left();

	if(enc_angle < -ONE_CLICK_ANGLE) {
		*mode = (*mode + 1) % num_mode;
		enc_angle = duty_click = 0.0f;
		if(direction == RIGHT)		Encoder_ResetCount_Right();
		else if(direction == LEFT)	Encoder_ResetCount_Left();
	} else if(enc_angle > ONE_CLICK_ANGLE) {
		*mode = (*mode + (num_mode-1)) % num_mode;
		enc_angle = duty_click = 0.0f;
		if(direction == RIGHT)		Encoder_ResetCount_Right();
		else if(direction == LEFT)	Encoder_ResetCount_Left();
	} else {
		if(ABS(ABS(enc_angle) - ONE_CLICK_ANGLE / 2.f) < DEG2RAD(3.f)) {
			duty_click = 0.f;
		} else if(ABS(enc_angle) > ONE_CLICK_ANGLE / 2.f) {
			duty_click = ONE_CLICK_GAIN * (SIGN(enc_angle) * ONE_CLICK_ANGLE - duty_click);
		} else {
			duty_click = -ONE_CLICK_GAIN * enc_angle;
		}
		duty_click = SIGN(duty_click) * MIN(ONE_CLICK_DUTY_LIMIT, ABS(duty_click));
	}

	if(direction == RIGHT)		Motor_SetDuty_Right(duty_click);
	else if(direction == LEFT)	Motor_SetDuty_Left(duty_click);

//	printf("%2d %6.2f %5d\r\n", *mode, RAD2DEG(enc_angle), duty_click);
}

/* ---------------------------------------------------------------
	全リセット
--------------------------------------------------------------- */
static void resetAllParams( void )
{
	if(Control_GetMode() == FAULT) {
		Motor_BrakePWM();
	} else {
		Motor_StopPWM();
	}
	SuctionFan_Stop();
	CleanerFan_Stop();
	Sensor_TurnOffLED();
//	Sensor_SetFrequency(NONE);
	Encoder_ResetCount_Right();
	Encoder_ResetCount_Left();
	IMU_ResetGyroAngle_Z();
	Vehicle_ResetTimer();
	Vehicle_ResetStraight();
	Vehicle_ResetTurning();
	Vehicle_ResetIntegral();
	Vehicle_ResetTotalDistance();
	Control_ResetFilterDistance();
	Control_ResetEncoderDeviation();
	Control_ResetGyroDeviation();
	Control_ResetAngleDeviation();
//	Control_ResetSensorDeviation();
//	Control_ResetFrontSensorDeviation();
//	Wall_ResetEdgeDistance();
//	Wall_ResetEdgeMinDistance();
}

/* ---------------------------------------------------------------
	スタート準備
--------------------------------------------------------------- */
static int8_t resetStartPreparation( void )
{
	int8_t sw_side = -1;

	// 前壁センサが反応するまで待機
	Sensor_TurnOnLED();
	while( Switch_GetIsFrontSensor() != -1 || Switch_GetIsTOF() != -1 );
	while( 1 ) {
		if( Switch_GetIsFrontSensor() != -1 || Switch_GetIsTOF() != -1 ) {
			if( Switch_GetIsFrontSensor() != -1 ) {
				sw_side = Switch_GetIsFrontSensor();
			} else {
				sw_side = Switch_GetIsTOF();
			}
			break;
		} else {}
	}
	Sensor_TurnOffLED();

	LED_LightBinary(0x00);
	if(sw_side == FRONT) {
		LED_LightBinary(0x00);
	} else {
		if(sw_side == RIGHT) { LED_LightBinary(0x01); }
		else				 { LED_LightBinary(0x08);  }
		HAL_Delay(500);
		IMU_ResetReference();
		resetAllParams();
		LED_LightBinary(0x00);
	}
	return sw_side;
}

