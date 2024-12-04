#include "module.h"

#define TRX_BUFFER_SIZE 128		// 送受信バッファサイズ

static struct {
    volatile uint16_t 	head;
    volatile uint16_t	tail;
    volatile uint16_t	remain;
    volatile uint8_t	data[TRX_BUFFER_SIZE];
} tx_buffer = {0}, rx_buffer = {0}; // FIFOバッファ

/* ---------------------------------------------------------------
	UART1で1文字受信する関数
--------------------------------------------------------------- */
uint8_t Communicate_ReceiveTest( void )
{
	uint8_t data[1];
	HAL_UART_Receive( &huart1, (uint8_t*)data, sizeof(data), 1 );
	return (*data);
}

uint8_t Communicate_Receive1byte( void )
{
// head（DMACが受信データを書き込む位置）に新しく受信データが蓄積される．
// tailから読み出せばいい
// headがtailに追いつくとバッファオーバーフローとなり正しく読み出せない

	// この関数は多重に実行されるとまずいので割り込みを禁止する
	__disable_irq();

	// headの位置を読み取る
	rx_buffer.head = TRX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER( huart1.hdmarx );

	// バッファが空ならヌル文字を返す
	if( rx_buffer.head == rx_buffer.tail ) {
		// 割り込み許可
		__enable_irq();

		return '\0';
	} else;

	// バッファから一文字読み取り返す
	char ch;
	ch = rx_buffer.data[rx_buffer.tail++];
	if( rx_buffer.tail == TRX_BUFFER_SIZE ) rx_buffer.tail = 0;

	// 割り込み許可
	__enable_irq();

	return ch;
}

/* ---------------------------------------------------------------
	受信バッファを空にする関数
--------------------------------------------------------------- */
void Communicate_ClearReceiveBuffer( void )
{
	while(Communicate_Receive1byte() != '\0');
}

/* ---------------------------------------------------------------
	UART1で1文字送信する関数
--------------------------------------------------------------- */
void Communicate_TransmitTest( uint8_t ch )
{
	HAL_UART_Transmit( &huart1, &ch, 1, 1 );
}

void Communicate_Transmit1byte( uint8_t ch )
{
// headに新しく追加する
// tailは次に送信されるデータを指す
// バッファに空きが無い（headがtailに追いついた）場合は待機する

	// バッファ内データ数をカウントし，空きがない場合待機する
	// バッファフルで待機しているときには割り込みを許可するためにwhileループになっている
	while(1) {
		// この関数は多重に実行されるとまずいので割り込みを禁止する
		__disable_irq();

		// DMACを一時的に停止
		HAL_UART_AbortTransmit( &huart1 );

		// 転送データ残数を取得
		for(volatile uint8_t i = 0; i < 0xff; i++);	// 謎の待ち時間
		tx_buffer.remain = __HAL_DMA_GET_COUNTER( huart1.hdmatx );

		// バッファに空きがあればループから抜ける
		if( tx_buffer.remain != TRX_BUFFER_SIZE ) break;

		// DMAC動作再開
		HAL_UART_Transmit_DMA( &huart1, (uint8_t*)(tx_buffer.data), TRX_BUFFER_SIZE );

		// 割り込み許可
		__enable_irq();

		// バッファに空きができるまで待機（この間割り込みが発生してもよい）
		while( __HAL_DMA_GET_COUNTER( huart1.hdmatx ) == TRX_BUFFER_SIZE );
	}
	// ここの時点でDMACは停止，割り込みは禁止されている

	// 次に書き込もうとする領域がバッファ終端なら前に詰める
	if(tx_buffer.head == TRX_BUFFER_SIZE){
		int blank = TRX_BUFFER_SIZE - tx_buffer.remain; // バッファの空き数
		int i;
		for(i=0;i+blank<TRX_BUFFER_SIZE;i++){
			tx_buffer.data[i] = tx_buffer.data[i+blank];
		}
		tx_buffer.head = i;
	}

	// 1文字追加
	tx_buffer.data[tx_buffer.head++] = ch;
	tx_buffer.remain++;
	tx_buffer.tail = tx_buffer.head - tx_buffer.remain;

	// DMACの再設定
	HAL_UART_Transmit_DMA( &huart1, (uint8_t*)(&tx_buffer.data[tx_buffer.tail]), tx_buffer.remain );

	// 割り込み許可
	__enable_irq();
}

/* ---------------------------------------------------------------
	printfとscanfを使用するための設定
--------------------------------------------------------------- */
void Communicate_Initialize( void )
{
	// エラー時の割込み禁止
	__HAL_UART_DISABLE_IT( &huart1, UART_IT_PE );
	__HAL_UART_DISABLE_IT( &huart1, UART_IT_ERR );

	// 受信DMA動作開始
	HAL_UART_Receive_DMA( &huart1, (uint8_t*)(rx_buffer.data), TRX_BUFFER_SIZE );

	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
}

/* ---------------------------------------------------------------
	printfを使用するための設定
--------------------------------------------------------------- */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
//	Communicate_TransmitTest(ch);
	Communicate_Transmit1byte(ch);
	return 1;
}

/* ---------------------------------------------------------------
	scanfを使用するための設定
--------------------------------------------------------------- */
#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */
GETCHAR_PROTOTYPE
{
	volatile uint8_t ch;
	while((ch = Communicate_Receive1byte()) == '\0');
	return ch;
}
