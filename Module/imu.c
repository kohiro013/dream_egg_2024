#include "module.h"
#include "motion.h"
#include "MPU6500_Register.h"

#define G					(9.80665f)	// 重量加速度[m/s^2]
#define REFFERENCE_NUM		(100)		// 何回の平均をもってジャイロのリファレンス電圧とするか

// ジャイロ関連マクロ
#define GYRO_Z_SIGN			(1.f)		// ジャイロの出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define GYRO_Z_SENSITIVITY	(16.4f)

// 加速度計関連マクロ
#define ACCEL_X_SIGN		(1.f)		// 加速度計の出力の符号（自分の座標系に合った方向に、1.0fか－1.0fを掛けて修正する）
#define ACCEL_X_SENSITIVITY	(4.096f)

// ローカル関数宣言
void 	IMU_Write2byte( uint8_t , uint16_t );
uint8_t IMU_Read1byte( uint8_t );

// グローバル変数宣言
volatile static uint8_t imu_address = ACCEL_XOUT_H | 0x80;
volatile static uint8_t	imu_value[15];			// value[0]はダミーデータ

volatile static int16_t	accel_x_value;			// X軸加速度計の生データ
volatile static int32_t	accel_x_reference;		// X軸加速度計のリファレンス

volatile static int16_t	gyro_z_value;			// Z軸ジャイロの生データ
volatile static int32_t	gyro_z_reference;		// Z軸ジャイロのリファレンス
volatile static float	gyro_z;
volatile static float	angle_z;

/* ---------------------------------------------------------------
	IMUに1byte書き込む関数
--------------------------------------------------------------- */
void IMU_Write1byte( uint8_t addr , uint8_t data )
{
	uint8_t address = addr & 0x7f;

	HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &address, 1, 1);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)(&data), 1, 1);
	HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET);
}

/* ---------------------------------------------------------------
	IMUから1byte読み出す関数
--------------------------------------------------------------- */
uint8_t IMU_Read1byte( uint8_t addr )
{
	// 送信バッファに書き込み //
	uint8_t address = addr | 0x80;
	uint8_t value;

	HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &address, 1, 1);
	HAL_SPI_Receive(&hspi2, &value, 1, 1);
	HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET);

	return value;
}

/* ---------------------------------------------------------------
	IMUの動作確認関数（WHO_AM_I(0x47)を取得する）
--------------------------------------------------------------- */
uint8_t IMU_CheckWHOAMI( void )
{
	return IMU_Read1byte( WHO_AM_I );
}

/* ---------------------------------------------------------------
	MPU-6500の初期設定用関数
--------------------------------------------------------------- */
void IMU_Initialize( void )
{
	// 起動時にCSピンがLowになっていると初回の通信に失敗するためCSピンをHighにする
	HAL_GPIO_WritePin( SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET );
	HAL_Delay(10);

	IMU_Write1byte(PWR_MGMT_1, 0x80);	// MPU6500をリセット
	HAL_Delay(100);
	IMU_Write1byte(SIGNAL_PATH_RESET, 0x07);	// assert signal path reset
	HAL_Delay(100);
	IMU_Write1byte(PWR_MGMT_1, 0x00);	// MPU6500をリセット
	HAL_Delay(100);

	IMU_Write1byte(USER_CTRL, 0x10);	// disable I2C mode
	HAL_Delay(1);
	IMU_Write1byte(SMPLRT_DIV, 0x00);	// sample rate max, DLPF = 0
	HAL_Delay(1);


	// ジャイロの設定
	IMU_Write1byte(GYRO_CONFIG, 0x18);		// ジャイロのスケールを±2000deg/sに設定
	HAL_Delay(1);							// ジャイロのローパスフィルタをEableに設定
	// 加速度計の設定
	IMU_Write1byte(ACCEL_CONFIG, 0x10);		// 加速度計のスケールを±8gに設定
	HAL_Delay(1);
//	IMU_Write1byte(ACCEL_CONFIG2, 0x08);	// 加速度計のローパスフィルタをEableに設定
//	HAL_Delay(1);

	// DMAの開始
//	HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_RESET);
//	HAL_SPI_TransmitReceive_DMA( &hspi2, (uint8_t*)(&imu_address), (uint8_t*)imu_value, sizeof(imu_value)/sizeof(uint8_t) );
}

/* ---------------------------------------------------------------
	DMA送受信完了後のコールバック関数
--------------------------------------------------------------- */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
	HAL_GPIO_WritePin( SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET );

	accel_x_value = ( ((int16_t)imu_value[ 3]<<8) | ((int16_t)imu_value[ 4]&0x00ff) );
	gyro_z_value =  ( ((int16_t)imu_value[13]<<8) | ((int16_t)imu_value[14]&0x00ff) );

	HAL_GPIO_WritePin( SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive_DMA( &hspi2, (uint8_t*)(&imu_address), (uint8_t*)imu_value, sizeof(imu_value)/sizeof(uint8_t) );
}

/* ---------------------------------------------------------------
	IMUの更新関数
--------------------------------------------------------------- */
void IMU_Update( void )
{
	HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive( &hspi2, (uint8_t*)(&imu_address), (uint8_t*)imu_value, sizeof(imu_value)/sizeof(uint8_t), 1 );
	HAL_GPIO_WritePin( SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET );

	accel_x_value = ( ((int16_t)imu_value[ 3]<<8) | ((int16_t)imu_value[ 4]&0x00ff) );
	gyro_z_value =  ( ((int16_t)imu_value[13]<<8) | ((int16_t)imu_value[14]&0x00ff) );

	gyro_z	= GYRO_Z_SIGN * DEG2RAD(((int16_t)gyro_z_value - gyro_z_reference) / GYRO_Z_SENSITIVITY);
	angle_z	+= gyro_z * SYSTEM_PERIOD;
}

/* ---------------------------------------------------------------
	IMUのリファレンスを補正する関数
--------------------------------------------------------------- */
void IMU_ResetReference( void )
{
	int16_t i;

	for(i = 0; i < REFFERENCE_NUM; i++) {
		HAL_Delay(1);
		accel_x_reference += (int16_t)accel_x_value;
		gyro_z_reference += (int16_t)gyro_z_value;
	}
	accel_x_reference /= REFFERENCE_NUM;
	gyro_z_reference /= REFFERENCE_NUM;
}

/* ---------------------------------------------------------------
	X軸加速度計の加速度を取得する関数[m/s^2]
--------------------------------------------------------------- */
float IMU_GetAccel_X( void )
{
	return ACCEL_X_SIGN * G * (accel_x_value - accel_x_reference) / ACCEL_X_SENSITIVITY;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角速度を取得する関数[rad/s]
--------------------------------------------------------------- */
float IMU_GetGyro_Z( void )
{
//	return GYRO_Z_SIGN * DEG2RAD( (gyro_z_value - gyro_z_reference) / GYRO_Z_SENSITIVITY );
	return gyro_z;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角度を取得する関数[rad]
--------------------------------------------------------------- */
float IMU_GetGyroAngle_Z( void )
{
	return angle_z;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角度に入力する関数[rad]
--------------------------------------------------------------- */
void IMU_SetGyroAngle_Z( float rad )
{
	angle_z = rad;
}

/* ---------------------------------------------------------------
	Z軸ジャイロの角度をリセットする関数[rad/s]
--------------------------------------------------------------- */
void IMU_ResetGyroAngle_Z( void )
{
	angle_z = 0.f;
}

void IMU_OffsetGyroAngle_Z( void )
{
	angle_z -= Vehicle_GetAngle();
}

/* ---------------------------------------------------------------
	デバッグ用
--------------------------------------------------------------- */
void IMU_DebugPrintf( void )
{
	IMU_ResetReference();
	while( Communicate_Receive1byte() != _ESC ) {
		printf("%5d, %5ld, %6.3f\t| %5d, %5ld, %9.3f\r\n",
			accel_x_value, accel_x_reference, IMU_GetAccel_X(),
			gyro_z_value,  gyro_z_reference,  IMU_GetGyro_Z());
		HAL_Delay(100);
	}
}
