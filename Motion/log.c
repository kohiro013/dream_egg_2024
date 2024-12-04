#include "module.h"
#include "motion.h"

#define STR(var) #var   //引数にした変数を変数名を示す文字列リテラルとして返すマクロ関数

#define NUM 	(3000)
#define THIN	(1)

typedef enum {
	Mode,
	Battery,
	Load,
	Duty_L, Duty_R,
	Target_A, Measure_A,
	Target_V, Measure_V,
	Target_Omega, Measure_Omega,
	Target_D, Measure_D,
	Target_Theta, Measure_Theta,
	Sensor_FL, Sensor_SL, Sensor_SR, Sensor_FR,
	Edge_SL, Edge_SR,
	Control_Encoder,
	Control_Gyro, Control_Angle, Control_Sensor,
	Gap,
	Dummy0, Dummy1, Dummy2, Dummy3, Dummy4, Dummy5,
	NUM_HEADER
} header;

volatile static uint16_t	count 					= 0;
volatile static int16_t		data[NUM][NUM_HEADER]	= {0};

extern float Vehicle_GetTorque( int8_t );
extern float Vehicle_GetRPM( int8_t );

void Log_TestData(void)
{
	for(int8_t i = 0; i < NUM; i++) {
		for(int8_t j = 0; j < NUM_HEADER; j++) {
			data[i][j]  = (int16_t)i * j * 10;
		}
	}

	for(int8_t i = 0; i < NUM; i++) {
		for(int8_t j = 0; j < NUM_HEADER; j++) {
			printf("%6d, ", data[i][j]);
		}
		printf("\r\n");
	}
	printf("\r\n");

	for(int8_t i = 0; i < NUM; i++) {
		for(int8_t j = 0; j < NUM_HEADER; j++) {
			printf("%04x, ", data[i][j]);
		}
		printf("\r\n");
	}
	printf("\r\n");
}

/* ----------------------------------------------------------------------------------
	ログ情報をFlash領域に保存
-----------------------------------------------------------------------------------*/
void Log_Store( void )
{
	HAL_StatusTypeDef result;

//	Log_TestData();
//	Flash_WriteData(data, sizeof(data));
	if( !Flash_EraseData() ) return;
	HAL_FLASH_Unlock();

	uint32_t address = 0x08100000;
	 for (int i = 0; i < NUM; ++i) {
		for (int j = 0; j < NUM_HEADER; j += 8) {
			uint8_t buffer[16];
			for (int k = 0; k < 8; ++k) {
				uint16_t value = data[i][j + k];
				buffer[k * 2] = value & 0xFF;
				buffer[k * 2 + 1] = (value >> 8) & 0xFF;
//				printf("%04x, ", value);
			}
			result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, address, (uint32_t)buffer);
			if( result != HAL_OK ) {
				break;
			} else {}
			address += sizeof(buffer);
		}
//		printf("\r\n");
	}
//	 printf("\r\n");

	 HAL_FLASH_Lock();
}

/* ----------------------------------------------------------------------------------
	ログ情報をFlash領域から読み込む
-----------------------------------------------------------------------------------*/
void Log_Load( void )
{
	Flash_ReadData((uint8_t*)data, sizeof(data)/sizeof(uint8_t));
}

void Log_WriteRecodeData( void )
{
	volatile static int thin_count = 0;

	if( Control_GetMode() > NONE ) {
		if( count < NUM && thin_count == 0 ) {
			data[count][Mode]  			 = (int16_t)Control_GetMode();
			data[count][Battery]  		 = (int16_t)(Battery_GetVoltage()*100.f);
			data[count][Load]  			 = (int16_t)Interrupt_GetDuty();
			data[count][Duty_L]  		 = (int16_t)Vehicle_GetDuty_Left();
			data[count][Duty_R]  		 = (int16_t)Vehicle_GetDuty_Right();
			data[count][Target_A]  		 = (int16_t)Vehicle_GetAcceleration();
			data[count][Measure_A]  	 = (int16_t)IMU_GetAccel_X();
			data[count][Target_V]  		 = (int16_t)Vehicle_GetVelocity();
			data[count][Measure_V]  	 = (int16_t)Control_GetFilterVelocity();
			data[count][Target_Omega]  	 = (int16_t)(RAD2DEG(Vehicle_GetAngularVelocity())*10.f);
			data[count][Measure_Omega]   = (int16_t)(RAD2DEG(IMU_GetGyro_Z())*10.f);
			data[count][Target_D] 		 = (int16_t)(Vehicle_GetDistance()*10.f);
			data[count][Measure_D] 		 = (int16_t)(Control_GetFilterDistance()*10.f);
			data[count][Target_Theta] 	 = (int16_t)(RAD2DEG(Vehicle_GetAngle())*10.f);
			data[count][Measure_Theta] 	 = (int16_t)(RAD2DEG(IMU_GetGyroAngle_Z())*10.f);

			data[count][Sensor_FL] 		 = (int16_t)(TOF_GetDistance(RIGHT)*10.f);
			data[count][Sensor_SL] 		 = (int16_t)(TOF_GetDistance(FRONT)*10.f);
			data[count][Sensor_SR] 		 = (int16_t)(TOF_GetDistance(LEFT)*10.f);
			data[count][Sensor_FR] 		 = (int16_t)(TOF_GetDistance(REAR)*10.f);

			data[count][Edge_SL] 		 = (int16_t)(Vehicle_GetGlobalX()*10.f);
			data[count][Edge_SR] 		 = (int16_t)(Vehicle_GetGlobalY()*10.f);
//			data[count][Gap] 			 = (int16_t)(RAD2DEG(Vehicle_GetYaw())*10.f);
			data[count][Gap] 			 = (int16_t)(RAD2DEG(TOF_GetTheta(LEFT))*10.f);

//			data[count][Sensor_FL] 		 = (int16_t)(Wall_GetDistance(FRONT + LEFT)*10.f);
//			data[count][Sensor_SL] 		 = (int16_t)(Wall_GetDistance(LEFT)*10.f);
//			data[count][Sensor_SR] 		 = (int16_t)(Wall_GetDistance(RIGHT)*10.f);
//			data[count][Sensor_FR] 		 = (int16_t)(Wall_GetDistance(FRONT + RIGHT)*10.f);
//			data[count][Edge_SL] 		 = (int16_t)Wall_GetEdge(LEFT);
//			data[count][Edge_SR] 		 = (int16_t)Wall_GetEdge(RIGHT);

			if(Control_GetMode() == FWALL) {
				data[count][Control_Encoder] = (int16_t)(Control_GetFrontWallVelocityDeviationValue());
				data[count][Control_Sensor]	 = (int16_t)(Control_GetFrontWallAngularDeviationValue()*10.f);
			} else {
				data[count][Control_Encoder] = (int16_t)(Control_GetEncoderDeviationValue());
				data[count][Control_Sensor]	 = (int16_t)(Control_GetSensorDeviationValue()*10.f);
			}
			data[count][Control_Gyro] 	 = (int16_t)(Control_GetGyroDeviationValue()*10.f);
			data[count][Control_Angle] 	 = (int16_t)(Control_GetAngleDeviationValue()*10.f);
//			data[count][Gap] 			 = (int16_t)(Vehicle_GetGap()*10.f);

			count++;
		} else;
		thin_count = (thin_count + 1) % THIN;
	} else;
}

void Log_ReadRecodeData( void )
{
	int16_t		log_read[NUM_HEADER] = {0};

	Log_Load();
/*	for(int8_t i = 0; i < NUM; i++) {
		for(int8_t j = 0; j < NUM_HEADER; j++) {
			printf("%6d, ", data[i][j]);
		}
		printf("\r\n");
	}
	printf("\r\n");

	printf("\r\n");
	for(int8_t i = 0; i < NUM; i++) {
		for(int8_t j = 0; j < NUM_HEADER; j++) {
			printf("%04x, ", data[i][j]);
		}
		printf("\r\n");
	}
	printf("\r\n");
*/
	printf("Time");
//	for( uint8_t i = 0; i < NUM_HEADER; i++ ) {
	for( uint8_t i = 0; i < Dummy0; i++ ) {
		printf(", ");
		switch(i) {
			case Mode:			 printf("%s", STR(Mode)); 			break;
			case Battery: 		 printf("%s", STR(Battery));		break;
			case Load:			 printf("%s", STR(Load)); 			break;
			case Duty_L:		 printf("%s", STR(Duty_L)); 		break;
			case Duty_R:		 printf("%s", STR(Duty_R)); 		break;
			case Target_A:		 printf("%s", STR(Target_A)); 		break;
			case Measure_A:		 printf("%s", STR(Measure_A)); 		break;
			case Target_V:		 printf("%s", STR(Target_V)); 		break;
			case Measure_V:		 printf("%s", STR(Measure_V)); 		break;
			case Target_Omega:	 printf("%s", STR(Target_Omega)); 	break;
			case Measure_Omega:	 printf("%s", STR(Measure_Omega)); 	break;
			case Target_D:		 printf("%s", STR(Target_D)); 		break;
			case Measure_D:		 printf("%s", STR(Measure_D)); 		break;
			case Target_Theta:	 printf("%s", STR(Target_Theta)); 	break;
			case Measure_Theta:	 printf("%s", STR(Measure_Theta)); 	break;
			case Sensor_FL:		 printf("%s", STR(Sensor_FL)); 		break;
			case Sensor_SL:		 printf("%s", STR(Sensor_SL)); 		break;
			case Sensor_SR:		 printf("%s", STR(Sensor_SR)); 		break;
			case Sensor_FR:		 printf("%s", STR(Sensor_FR)); 		break;
			case Edge_SL:		 printf("%s", STR(Edge_SL)); 		break;
			case Edge_SR:		 printf("%s", STR(Edge_SR)); 		break;
			case Control_Encoder:printf("%s", STR(Control_Encoder));break;
			case Control_Gyro:	 printf("%s", STR(Control_Gyro)); 	break;
			case Control_Angle:	 printf("%s", STR(Control_Angle)); 	break;
			case Control_Sensor: printf("%s", STR(Control_Sensor)); break;
			case Gap:			 printf("%s", STR(Gap)); 			break;
			default:			 									break;
		}
	}
	printf("\r\n");

	for( uint32_t num = 0; num < NUM; num++ ) {
		for( uint8_t i = 0; i < Dummy0; i++ ) {
//		for( uint8_t i = 0; i < NUM_HEADER; i++ ) {
			log_read[i] = data[num][i];
		}

		printf("%5.3f", num*SYSTEM_PERIOD*THIN);
		for( uint8_t i = 0; i < Dummy0; i++ ) {
//		for( uint8_t i = 0; i < NUM_HEADER; i++ ) {
			printf(", %6d", log_read[i]);
		}
		printf("\r\n");
	}
	printf("%c\r\n", 0x1b);
	Communicate_ClearReceiveBuffer();
}
