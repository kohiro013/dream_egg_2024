#include "module.h"

// LED関連マクロ
#define LED_YELLOW_ON()		HAL_GPIO_WritePin( LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET)		// 黄LEDを点灯する
#define LED_YELLOW_OFF()	HAL_GPIO_WritePin( LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET)	// 黄LEDを消灯する
#define LED_YELLOW_TOGGLE()	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin)					// この関数を呼ぶたびに黄LEDの点灯と消灯を切り替える
#define LED_RED_ON()		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)		// 赤LEDを点灯する
#define LED_RED_OFF()		HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)	// 赤LEDを消灯する
#define LED_RED_TOGGLE()	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)					// この関数を呼ぶたびに赤LEDの点灯と消灯を切り替える
#define LED_GREEN_ON()		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)		// 緑LEDを点灯する
#define LED_GREEN_OFF()		HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)	// 緑LEDを消灯する
#define LED_GREEN_TOGGLE()	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)					// この関数を呼ぶたびに緑LEDの点灯と消灯を切り替える
#define LED_BLUE_ON()		HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)		// 青LEDを点灯する
#define LED_BLUE_OFF()		HAL_GPIO_WritePin( LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)	// 青LEDを消灯する
#define LED_BLUE_TOGGLE()	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)					// この関数を呼ぶたびに青LEDの点灯と消灯を切り替える

// WS2812C関連マクロ
#define COMPARE_LED_OFF		(16)	// オフ時のカウント値
#define COMPARE_LED_ON		(33)	// オン時のカウント値

#define NUM_LED				(1)				// LEDの数
#define NUM_OFFSET 			(0)
#define NUM_COLOR_BIT		(8 * NUM_LED)	// RGB3色 x 8bit
#define BRIGHTNESS 			(0x0f)			// LEDの明るさ

typedef enum {	// LEDの色列挙
	BLACK	= 0,
	RED		= 1,
	GREEN	= 2,
	BLUE	= 3,
	YELLOW	= 4,
	SKY 	= 5,
	PURPLE	= 6,
	WHITE 	= 7,
	NUM_COLOR,
} t_led_color;
/*
const static uint8_t COLOR_MAP[NUM_COLOR][3] = {
	{0x00, 		 0x00, 		 0x00		},	// NONE
	{BRIGHTNESS, 0x00, 		 0x00		},	// RED
	{0x00, 		 BRIGHTNESS, 0x00		},	// GREEN
	{0x00, 		 0x00, 		 BRIGHTNESS	},	// BLUE
	{BRIGHTNESS, BRIGHTNESS, 0x00		},	// YELLOW
	{0x00, 		 BRIGHTNESS, BRIGHTNESS	},	// SKY
	{BRIGHTNESS, 0x00, 		 BRIGHTNESS	},	// PURPLE
	{BRIGHTNESS, BRIGHTNESS, BRIGHTNESS	},	// WHITE
};
*/
// LEDのバイナリ表示用構造体
typedef union {
	uint8_t byte;
	struct {
		uint8_t led0			:1;
		uint8_t led1			:1;
		uint8_t led2			:1;
		uint8_t led3			:1;
		uint8_t dummy			:4;
	} bit;
} t_led_num;

// グローバル変数群
volatile static t_led_num 		led_num;
volatile static uint16_t 		led_interval = 0;

volatile static uint16_t 		led_pattern[NUM_LED * NUM_COLOR_BIT * 2] = {0};	// 後半はリセット時間確保のため0固定
volatile static uint8_t 		led_color = 0;

/* ---------------------------------------------------------------
	WS2812Cに点灯指令を送る関数
--------------------------------------------------------------- */
void LED_StartPWM( void )
{
	HAL_TIM_PWM_Start_DMA(&htim17, TIM_CHANNEL_1, (uint32_t *)led_pattern, sizeof(led_pattern)/sizeof(led_pattern[0]));
}

/* ---------------------------------------------------------------
	WS2812Cの初期設定関数
--------------------------------------------------------------- */
void LED_Initialize( void )
{
	// LEDの消灯
	for(uint8_t i = 0; i < NUM_LED * NUM_COLOR_BIT; i++) {
		led_pattern[i] = COMPARE_LED_OFF;
	}
	LED_StartPWM();
}

/* ---------------------------------------------------------------
	WS2812Cのリセット関数
--------------------------------------------------------------- */
void LED_Reset( void )
{
	for(uint8_t i = 0; i < sizeof(led_pattern)/sizeof(led_pattern[0]); i++) {
		led_pattern[i] = 0;
	}
	LED_StartPWM();
}

/* ---------------------------------------------------------------
	指定のLEDにRGB値を入力する関数
--------------------------------------------------------------- */
void LED_SetRGB( uint8_t num, uint8_t byte_r, uint8_t byte_g, uint8_t byte_b )
{
	for(int8_t i = 0; i < 8; i++) {
		led_pattern[NUM_COLOR_BIT * num + 7 - i]  = ((byte_g&(0x01 << i)) == (0x01 << i)) ? COMPARE_LED_ON : COMPARE_LED_OFF;
		led_pattern[NUM_COLOR_BIT * num + 15 - i] = ((byte_r&(0x01 << i)) == (0x01 << i)) ? COMPARE_LED_ON : COMPARE_LED_OFF;
		led_pattern[NUM_COLOR_BIT * num + 23 - i] = ((byte_b&(0x01 << i)) == (0x01 << i)) ? COMPARE_LED_ON : COMPARE_LED_OFF;
	}
}

/* ---------------------------------------------------------------
	割り込み内でLEDの点灯時間を更新する関数
--------------------------------------------------------------- */
void LED_Update( void )
{
//	volatile static uint16_t 	led_status_hash = 0;
//	volatile uint16_t 			led_change_hash = 0;
	volatile static uint16_t 	led_timer = 0;

	if( led_interval > 0 ) {
		led_timer++;
		if( led_timer > led_interval ) {
			led_timer = 0;
			if( led_num.bit.led0 ) 	LED_YELLOW_TOGGLE();
			else					LED_YELLOW_OFF();
			if( led_num.bit.led1 ) 	LED_RED_TOGGLE();
			else					LED_RED_OFF();
			if( led_num.bit.led2 ) 	LED_GREEN_TOGGLE();
			else					LED_GREEN_OFF();
			if( led_num.bit.led3 ) 	LED_BLUE_TOGGLE();
			else					LED_BLUE_OFF();
		} else;
	} else {
		if( led_num.bit.led0 ) 	LED_YELLOW_ON();
		else					LED_YELLOW_OFF();
		if( led_num.bit.led1 ) 	LED_RED_ON();
		else					LED_RED_OFF();
		if( led_num.bit.led2 ) 	LED_GREEN_ON();
		else					LED_GREEN_OFF();
		if( led_num.bit.led3 ) 	LED_BLUE_ON();
		else					LED_BLUE_OFF();
	}
}

/* ---------------------------------------------------------------
	バイナリ指定でLEDを点灯させる関数
--------------------------------------------------------------- */
void LED_LightBinary( uint8_t num )
{
	led_interval = 0;
	led_num.byte = num;
}

/* ---------------------------------------------------------------
	バイナリ指定でLEDをトグル点灯させる関数
--------------------------------------------------------------- */
void LED_ToggleBinary( uint8_t num )
{
	led_interval = 0;
	led_num.byte ^= num;
}

/* ---------------------------------------------------------------
	バイナリ指定でLEDを指定時間点滅させる関数
--------------------------------------------------------------- */
void LED_TimerBinary( uint8_t num, uint16_t ms )
{
	led_interval = ms;
	led_num.byte = num;
}
