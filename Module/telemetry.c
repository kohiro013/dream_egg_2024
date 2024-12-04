#include "module.h"
#include "application.h"

enum {
	TELEM_ERROR		= 0,
	TELEM_READY 	= 1,
	TELEM_HEADER 	= 2,
	TELEM_LENGTH	= 3,
	TELEM_DATA		= 4,
	TELEM_CRC		= 5,
};

// 受信関連マクロ
#define RX_BUFFER_SIZE 		(1024)
#define DMA_WRITE_PTR 		((RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx)) % (RX_BUFFER_SIZE))
#define RX_HEADER			('>')

// 送信関連マクロ
#define TX_BUFFER_SIZE 		(12)
#define TX_HEADER			('<')
#define TX_DATA_LENGTH 		(TX_BUFFER_SIZE - 3)

// 受信バッファ
volatile static uint8_t 	rx_buffer[RX_BUFFER_SIZE];
volatile static uint32_t 	rd_ptr = 0;

volatile static int8_t		mode = TELEM_READY;
volatile static uint16_t 	length = 0xffff;
volatile static uint8_t		checksum = 0;
volatile static uint8_t		data[RX_BUFFER_SIZE];

// 送信バッファ
volatile static uint8_t 	tx_buffer[TX_BUFFER_SIZE];


void Telemetry_GetData( uint8_t* value, uint16_t* size )
{
	*size = length;
	for( uint16_t i = 0; i < length; i++ ) {
		value[i] = data[i];
	}
}

/* ---------------------------------------------------------------
	テレメトリのデコード関数
--------------------------------------------------------------- */
int8_t Telemetry_Decode( void )
{
	static uint16_t count = 0;

	while( rd_ptr != DMA_WRITE_PTR ) {
		uint8_t ch = rx_buffer[rd_ptr++];
		rd_ptr %= RX_BUFFER_SIZE;
//		printf("%02x ", ch);

		switch( mode ) {
			case TELEM_READY:
				if( ch == RX_HEADER ) {
					mode = TELEM_HEADER;
					length = 0xffff;
					checksum = 0;
					for( uint16_t i = 0; i < RX_BUFFER_SIZE; i++ ) {
						data[i] = 0xff;
					}
					count = 0;
//					printf("header: %c, ", ch);
				} else {}
			break;

			case TELEM_HEADER:
				if( length == 0xffff) {
					length = ((uint16_t)(ch << 8) &0xff00);
				} else {
					length |= (uint16_t)ch &0x00ff;

					if( length > RX_BUFFER_SIZE ) {
						mode = TELEM_READY;
					} else {
						mode = TELEM_DATA;
					}
//					printf("length: %02d, ", length);
				}
			break;

			case TELEM_DATA:
				data[count] = ch;
				if( count == length - 1 ) {
					mode = TELEM_CRC;
/*					printf("data: ");
					for( uint16_t i = 0; i < length; i++ ) {
						printf("%02x ", data[i]);
					}
					printf(", ");
*/				} else {
					count ++;
				}
			break;

			case TELEM_CRC:
				for(uint16_t i = 0; i < length; i++) {
					checksum = (checksum + data[i]) &0xff;
				}
				mode = TELEM_READY;
				if( checksum == ch ) {
//					printf("checksum: %02x = %02x\r\n", checksum, ch);
					if( memcmp( (uint8_t*)data, "START", 5 ) == 0 ) {
//						printf("start!\r\n");
						return 1;
					} else if( memcmp( (uint8_t*)data, "STOP", 4 ) == 0 ) {
//						printf("stop!\r\n");
						return 2;
					} else if( memcmp( (uint8_t*)data, "RETURN", 6 ) == 0 ) {
//						printf("Return mode!\r\n");
						return 3;
					} else if( memcmp( (uint8_t*)data, "RC", 2 ) == 0 ) {
//						printf("RC mode!\r\n");
						return 4;
					} else {
/*						printf("path: ");
						for( uint16_t i = 0; i < length; i++ ) {
							printf("%02d ", data[i]);
						}
						printf("\n\r");
*///						Route_SetPath( (uint8_t*)data, length );
						return 5;
					}
				} else {}
			break;
		}
	}

	if( HAL_UART_GetError(&huart6) == HAL_UART_ERROR_ORE ) {
		__HAL_UART_CLEAR_FLAG(&huart6, UART_CLEAR_OREF);
	} else {}

	return 0;
}

/* ---------------------------------------------------------------
	テレメトリの送信関数
--------------------------------------------------------------- */
void Telemetry_Transmit( void )
{
	uint16_t x = (uint16_t)Vehicle_GetGlobalX();
	uint16_t y = (uint16_t)Vehicle_GetGlobalY();

	tx_buffer[ 0] = TX_HEADER;
	tx_buffer[ 1] = TX_DATA_LENGTH;
	tx_buffer[ 2] = (uint8_t)((x >> 8) &0xff);	// X座標
	tx_buffer[ 3] = (uint8_t)(x &0xff);
	tx_buffer[ 4] = (uint8_t)((y >> 8) &0xff);	// Y座標
	tx_buffer[ 5] = (uint8_t)(y &0xff);
	tx_buffer[ 6] = (uint8_t)(Vehicle_GetYaw() / (2 * PI) * 0xff);	// 絶対座標角度
	tx_buffer[ 7] = (uint8_t)(Control_GetFilterVelocity() / 100.f);	// 車速[m/s * 10]
	tx_buffer[ 8] = (uint8_t)(Vehicle_GetVoltage() * 10.f);			// バッテリ電圧[V * 10]
	tx_buffer[ 9] = 0;	// 走行状態
	tx_buffer[10] = (uint8_t)(Control_GetMode() == FAULT);	// エラー

	uint8_t checksum = 0;
	for( uint8_t i = 0; i < TX_DATA_LENGTH; i++ ) {
		checksum = (checksum + tx_buffer[i + 2]) &0xff;
	}
	tx_buffer[11] = checksum;

	// 送信DMA動作開始
	HAL_UART_Transmit_DMA( &huart6, (uint8_t*)(tx_buffer), TX_BUFFER_SIZE );
}

/* ---------------------------------------------------------------
	printfとscanfを使用するための設定
--------------------------------------------------------------- */
void Telemetry_Initialize( void )
{
	// エラー時の割込み禁止
	__HAL_UART_DISABLE_IT( &huart6, UART_IT_PE );
	__HAL_UART_DISABLE_IT( &huart6, UART_IT_ERR );

	// 受信DMA動作開始
	HAL_UART_Receive_DMA( &huart6, (uint8_t*)(rx_buffer), RX_BUFFER_SIZE );
}

