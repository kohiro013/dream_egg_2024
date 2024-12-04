#include "module.h"
#include "VL53Lx_api.h"
#include "motion.h"

#define WRITE_TOF_EN_RIGHT(x)	HAL_GPIO_WritePin(TOF_EN3_GPIO_Port, TOF_EN3_Pin, x)
#define WRITE_TOF_EN_FRONT(x)	HAL_GPIO_WritePin(TOF_EN0_GPIO_Port, TOF_EN0_Pin, x)
#define WRITE_TOF_EN_LEFT(x)	HAL_GPIO_WritePin(TOF_EN1_GPIO_Port, TOF_EN1_Pin, x)
#define WRITE_TOF_EN_REAR(x)	HAL_GPIO_WritePin(TOF_EN2_GPIO_Port, TOF_EN2_Pin, x)

// グローバル変数宣言
static VL53LX_Dev_t		dev_right, dev_front, dev_left, dev_rear;
static VL53LX_DEV		Dev[4] = {&dev_right, &dev_front, &dev_left, &dev_rear};
volatile static int16_t distance[4];
volatile static int16_t distance_old[4];
volatile static float 	theta[4];

static void set_xshut_pin( uint8_t device, GPIO_PinState pin_state )
{
	switch(device) {
		case RIGHT:	WRITE_TOF_EN_RIGHT(pin_state);	break;
		case FRONT:	WRITE_TOF_EN_FRONT(pin_state);	break;
		case LEFT:	WRITE_TOF_EN_LEFT(pin_state);	break;
		case REAR:	WRITE_TOF_EN_REAR(pin_state);	break;
		default:									break;
	}
}

static void reset_all_sensors( void )
{
	for(int8_t i = 0; i < 4; i++) {
		set_xshut_pin(i, GPIO_PIN_RESET);
	}
	HAL_Delay(5);

	for(int8_t i = 0; i < 4; i++) {
		set_xshut_pin(i, GPIO_PIN_SET);
	}
	HAL_Delay(5);
}

/* ---------------------------------------------------------------
	TOFセンサの初期設定関数
--------------------------------------------------------------- */
void TOF_Initialize( void )
{
	int 			status = 0;
//	uint8_t 		byteData;
//	uint16_t 		wordData;

	// リセット
	reset_all_sensors();
	for(int8_t i = 0; i < 4; i++) {
		set_xshut_pin(i, GPIO_PIN_RESET);
	}

	// TOFの初期設定
	for(int8_t i = 1; i < 2; i++) {
		set_xshut_pin(i, GPIO_PIN_SET);
		HAL_Delay(5);

		Dev[i]->I2cHandle = &hi2c2;
		Dev[i]->I2cDevAddr = 0x52;

		status = VL53LX_WaitDeviceBooted(Dev[i]);
		if(status){
			printf("VL53LX_WaitDeviceBooted failed: error = %d\n\r", status);
			while(1);
		}

		status = VL53LX_DataInit(Dev[i]);
		if(status){
			printf("VL53LX_DataInit failed: error = %d\n\r", status);
			while(1);
		}

		set_xshut_pin(i, GPIO_PIN_RESET);
	}

	// I2Cのアドレスを変更
	// XSHUTをオフにするとアドレスがリセットされるのでオフにしないこと
	// アドレスはAPI内で半分になるので2以上離すこと
	for(int8_t i = 1; i < 2; i++) {
		set_xshut_pin(i, GPIO_PIN_SET);
		HAL_Delay(5);

		status = VL53LX_SetDeviceAddress(Dev[i], 0x40 + 2 * i);
//		printf("Status: %d, ID: %02x →　%02x\r\n", status, Dev[i]->I2cDevAddr, 0x40 + 2 * i);
		if(status){
			printf("VL53LX_SetDeviceAddress failed: error = %d\n\r", status);
			while(1);
		}
		Dev[i]->I2cDevAddr = 0x40 + 2 * i;

//		VL53LX_RdByte(Dev[i], 0x0001, &byteData);
//		printf("VL53L4CX Model_ID: %02x (%02x)\n\r", (byteData * 2) &0x7f, Dev[i]->i2c_slave_address);
	}

	// TOFの測距開始
	for(int8_t i = 1; i < 2; i++) {
//		status = VL53LX_SetMeasurementTimingBudgetMicroSeconds(Dev[i], 10000);

//		printf("Ranging loop starts : %02x\n\r", Dev[i]->I2cDevAddr);
		status = VL53LX_StartMeasurement(Dev[i]);
		if(status){
			printf("VL53LX_StartMeasurement failed: error = %d\n\r", status);
			while(1);
		}
	}
//	printf("\r\n");
}

/* ---------------------------------------------------------------
	TOFの更新関数
--------------------------------------------------------------- */
void TOF_Update( void )
{
	int 						status = 0;
	uint8_t 					pMeasurementDataReady;
	VL53LX_MultiRangingData_t 	MultiRangingData;
	VL53LX_MultiRangingData_t 	*pMultiRangingData = &MultiRangingData;
	static float mileage[4] = {0.f, 0.f, 0.f, 0.f};

	volatile static int8_t i = 0;

	status = VL53LX_GetMeasurementDataReady(Dev[i], &pMeasurementDataReady);
	if(!status && pMeasurementDataReady == 0x01){
		status = VL53LX_GetMultiRangingData(Dev[i], pMultiRangingData);
		if(status == 0) {
			distance_old[i] = distance[i];
			distance[i] = pMultiRangingData->RangeData[0].RangeMilliMeter;
			status = VL53LX_ClearInterruptAndStartMeasurement(Dev[i]);

			float d = Vehicle_GetDistance();
			float l = distance[i] - distance_old[i];
			// TOF測距距離の変化量が15mm以内で、測距
			if( ABS(l) < 15.f && d - mileage[i] > 5.f ) {
				arm_atan2_f32(ABS(l), d - mileage[i], (float*)(&theta[i]));
				theta[i] *= SIGN(l);
				if( ABS(RAD2DEG(theta[i])) < 10.f && i == LEFT ) {
					IMU_SetGyroAngle_Z(-theta[i]);
				} else {}
			} else {}
			mileage[i] = d;
		} else {}
	}

	i = (i + 1) % 4;
}

/* ---------------------------------------------------------------
	TOFセンサで取得した距離を取得する関数
--------------------------------------------------------------- */
int16_t TOF_GetDistance( int8_t direction )
{
	if( direction < 0 || 3 < direction ) {
		return 0;
	} else {}
	return distance[direction];
}

/* ---------------------------------------------------------------
	TOFセンサで推定した角度を取得する関数
--------------------------------------------------------------- */
float TOF_GetTheta( int8_t direction )
{
	if( direction < 0 || 3 < direction ) {
		return 0;
	} else {}
	return theta[direction];
}

/* ---------------------------------------------------------------
	デバッグ用
--------------------------------------------------------------- */
void TOF_DebugPrintf( void )
{
	while( Communicate_Receive1byte() != _ESC ) {
		printf("RIGHT: %4d mm, FRONT: %4d mm, LEFT: %4d mm, REAR: %4d mm\r\n",
				distance[RIGHT], distance[FRONT], distance[LEFT], distance[REAR]);
		HAL_Delay(100);
	}
}
