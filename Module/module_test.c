#include "module.h"

/* ---------------------------------------------------------------
	各機能の動作確認用関数
--------------------------------------------------------------- */
void module_test( void )
{
	uint16_t	line 	  = 0;
	uint8_t		key;
	int16_t		duty_l	  = 0;
	int16_t		duty_r	  = 0;

	// IMuのリファレンスをリセット
	HAL_Delay(1000);
	IMU_ResetReference();

	// エンコーダのカウントをリセット
	Encoder_ResetCount_Left();
	Encoder_ResetCount_Right();

	// 壁センサ用LEDの点灯開始
	Sensor_TurnOnLED();

	// UARTの受信バッファをリセット
	Communicate_ClearReceiveBuffer();

	while( Communicate_Receive1byte() != _ESC ) {
		// 割り込み処理率を表示
		printf("<Boot Time> %8.3f[s]\r\n", Interrupt_GetBootTime()); line++;
		printf("<Interrupt> %3.1f[%%] (MAX : %3.1f[%%])\r\n",
				(float)Interrupt_GetDuty()/10.f, (float)Interrupt_GetDuty_Max()/10.f); line++;

		// モータを指定のDutyを表示
		printf("<PWM Duty> L: %4.1f[%%],  R: %4.1f[%%]\r\n",
				(float)duty_l/10.f, (float)duty_r/10.f); line++;

		// エンコーダの角度表示
		printf("<Encoder> L: %5.1f[deg],  R: %5.1f[deg]\r\n",
				RAD2DEG(Encoder_GetAngle_Left()), RAD2DEG(Encoder_GetAngle_Right())); line++;

		// バッテリー電圧の表示
		printf("<Battery> Battery : %3.2f[V]\r\n",
				Battery_GetVoltage()); line++;

		// 壁センサのAD値表示
		printf("<IR Sensor> FL: %4d, SL: %4d SR: %4d, FR: %4d\r\n",
				Sensor_GetValue(FRONT+LEFT), Sensor_GetValue(LEFT),
				Sensor_GetValue(RIGHT), Sensor_GetValue(FRONT+RIGHT)); line++;

		// TOFセンサの距離表示
		printf("<TOF> Right: %4d, Front: %4d Left: %4d, Rear: %4d\r\n",
				TOF_GetDistance(RIGHT), TOF_GetDistance(FRONT),
				TOF_GetDistance(LEFT), TOF_GetDistance(REAR)); line++;

		// IMU（加速度計とジャイロ）の計測値表示
		printf("<IMU> Accel_X: %5.3f[m/s^2], Gyro_Z: %6.3f[rad/s]\r\n",
				IMU_GetAccel_X(), IMU_GetGyro_Z()); line++;

		// モータのDuty入力
		key = Communicate_Receive1byte();
		switch( key ) {
			case '1': duty_l += 1;		break;
			case 'q': duty_l -= 1;		break;
			case '2': duty_l += 10;		break;
			case 'w': duty_l -= 10;		break;
			case '3': duty_l += 100;	break;
			case 'e': duty_l -= 100;	break;
			case '7': duty_r += 1;		break;
			case 'u': duty_r -= 1;		break;
			case '8': duty_r += 10;		break;
			case 'i': duty_r -= 10;		break;
			case '9': duty_r += 100;	break;
			case 'o': duty_r -= 100;	break;
			case 'r': // reset
				duty_l = duty_r = 0;
				break;
			case 0x1b: goto END; // [esc] exit
		}
		duty_l = SIGN(duty_l) * MIN( 950, ABS(duty_l) );
		duty_r = SIGN(duty_r) * MIN( 950, ABS(duty_r) );

		// モータを回転
		Motor_SetDuty_Left(duty_l);
		Motor_SetDuty_Right(duty_r);

		fflush(stdout);
		HAL_Delay(100);
		// 画面のクリア
		printf("%c[0J", 0x1b);
		printf("%c[%dA", 0x1b, line);
	}
	END:;
	printf("\r\n");

	// モータの停止
	Motor_StopPWM();

	// 壁センサ用LEDの点灯開始
	Sensor_TurnOffLED();
}
