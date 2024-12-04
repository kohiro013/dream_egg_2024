#include "module.h"

#define BATTERY_REFERENCE	(3.3f)
#define BATTERY_LIMIT		(10.5f)

#define	BATTERY_R1			(33.f)
#define	BATTERY_R2			(10.f)

/* ---------------------------------------------------------------
	バッテリの電圧を取得する関数
--------------------------------------------------------------- */
float Battery_GetVoltage( void )
{
	return (BATTERY_REFERENCE * ((BATTERY_R1 + BATTERY_R2) / BATTERY_R2) * (float)Sensor_GetBatteryValue()) / 4096.f;
}

/* ---------------------------------------------------------------
	バッテリの電圧制限関数
--------------------------------------------------------------- */
float Battery_GetLimitVoltage( void )
{
	return BATTERY_LIMIT;
}

void Battery_LimiterVoltage( void )
{
	volatile int	i;
	volatile float	battery_voltage_average;

	for( i = 0; i < 10; i++) {
		HAL_Delay(10);
		battery_voltage_average += Battery_GetVoltage();
	}
	battery_voltage_average /= 10;

	// USB接続時はバッテリー電圧が0Vになるため、制限電圧以下かつ数V以上で制限する
	if( 0.5f < battery_voltage_average && battery_voltage_average < BATTERY_LIMIT ) {
		LED_TimerBinary(0xff, 100);
		while( 1 );
	} else;
}
