
#include "application.h"

#define NUM_PATH	255

volatile uint8_t	num;
volatile t_path		path[NUM_PATH];


/* ----------------------------------------------------------------------------------
	パスの初期設定
-----------------------------------------------------------------------------------*/
void Path_Reset( void )
{
	for( num = 0; num < NUM_PATH-1; num++ ) {
		path[num].straight = 0;
		path[num].direction = -1;
		path[num].type = TURN_0;
	}
	num = 0;
}

/* ----------------------------------------------------------------------------------
	直線パスのセット
-----------------------------------------------------------------------------------*/
void Path_SetStraightSection( int8_t n )
{
	path[num].straight += n;
}

/* ----------------------------------------------------------------------------------
	ターンパスのセット
-----------------------------------------------------------------------------------*/
void Path_SetTurnSection( int8_t type, int8_t direction )
{
	path[num].direction = direction;
	path[num].type = type;
	num++;
}

/* ----------------------------------------------------------------------------------
	生成したパスの参照
-----------------------------------------------------------------------------------*/
t_path Path_GetSequence( uint8_t n )
{
	return path[n];
}

t_path Path_GetReturnSequence( uint8_t n )
{
	t_path return_path;
	if( (num-1) - n == 0 ) {
		return_path.straight	= path[0].straight;
		return_path.type		= GOAL;
		return_path.direction	= 0;

	} else if( (num-1) - n-1 < 0 ) {
		return_path.straight	= 0;
		return_path.type		= TURN_0;
		return_path.direction	= -1;

	} else {
		return_path.straight	= path[(num-1)-n].straight;

		if( path[(num-1)-n-1].type == TURN_45IN ) {
			return_path.type = TURN_45OUT;
		} else if( path[(num-1)-n-1].type == TURN_135IN ) {
			return_path.type = TURN_135OUT;
		} else if( path[(num-1)-n-1].type == TURN_45OUT ) {
			return_path.type = TURN_45IN;
		} else if( path[(num-1)-n-1].type == TURN_135OUT ) {
			return_path.type = TURN_135IN;
		} else {
			return_path.type = path[(num-1)-n-1].type;
		}

		if( path[(num-1)-n-1].direction == RIGHT ) {
			return_path.direction = LEFT;
		} else if( path[(num-1)-n-1].direction == LEFT ) {
			return_path.direction = RIGHT;
		} else {
			return_path.direction = -1;
		}
	}
	return return_path;
}

/* ----------------------------------------------------------------------------------
	生成したパス数の参照
-----------------------------------------------------------------------------------*/
uint8_t Path_GetSequenceNumber( void )
{
	uint8_t i;
	for( i = 0; i < NUM_PATH-1; i++ ) {
		// ゴールパスがある場合
		if( path[i].type == GOAL ) {
			return i + 1;
		} else;
	}
	// ゴールパスがない場合は0を返す
	return 0;
}

/* ----------------------------------------------------------------------------------
	掃除用のパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertCleaner( void )
{
	for( uint8_t i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == GOAL ) {
			num = i + 1;
			break;
		} else;

		if( path[i].type == TURN_90 ) {
			path[i].straight ++;
			path[i].type = TURN_0;
			path[i+1].straight ++;
		} else if( path[i].type == TURN_180 ) {
			path[i].straight ++;
			path[i].type = TURN_0;
			path[i+1].straight ++;
		} else {}
	}
}

/* ----------------------------------------------------------------------------------
	大回りターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurnLarge( void )
{
	for( uint8_t i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == GOAL ) {
			num = i + 1;
			break;
		} else;

		if( path[i].type == TURN_90 ) {
			if( (path[i].straight > 0) && (path[i+1].straight > 0) ) {
				path[i].straight--;
				path[i].type = TURN_90L;
				path[i+1].straight--;
			} else;
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	180度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn180( void )
{
	for( uint8_t i = 0; i < NUM_PATH-1; i++ )	{
		if( path[i].type == GOAL ) {
			num = i + 1;
			break;
		} else;

		if( (path[i].straight > 0) && (path[i+1].straight == 0) && (path[i+2].straight > 0) ) {
			if( (path[i].type == TURN_90) && (path[i+1].type == TURN_90) && (path[i].direction == path[i+1].direction) ) {
				path[i].straight--;
				path[i].type = TURN_180;
				path[i+2].straight--;
				for( uint8_t j = i + 1; j < NUM_PATH; j++ ) {
					if( path[j].type == GOAL ) {
						break;
					} else;

					path[j].straight = path[j+1].straight;
					path[j].direction = path[j+1].direction;
					path[j].type = path[j+1].type;
				}
			} else;
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	斜め侵入型45度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn45in( uint8_t i )
{
	if( (path[i].straight > 0) && (path[i+1].straight == 0 )) {
		if( (path[i].type == TURN_90) && (path[i+1].type == TURN_90)) {
			return( path[i].direction != path[i+1].direction );
		} else;
	}else;
	return(false);
}

/* ----------------------------------------------------------------------------------
	斜め脱出型45度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn45out( uint8_t i )
{
	if( (path[i].straight == 0) && (path[i+1].straight > 0) ) {
		if( (path[i-1].type == TURN_90) && (path[i].type == TURN_90) ) {
			return( path[i-1].direction != path[i].direction );
		} else if( (path[i-1].type == TURN_45IN) && (path[i].type == TURN_90) ) {
			return( path[i-1].direction != path[i].direction );
		} else;
	} else;
	return( false );
}

/* ----------------------------------------------------------------------------------
	斜め侵入型135度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn135in( uint8_t i )
{
	if( (path[i].straight > 0) && (path[i+1].straight == 0) && (path[i+2].straight == 0) ) {
		if( (path[i].type == TURN_90) && (path[i+1].type == TURN_90) ) {
			return( path[i].direction == path[i+1].direction );
		} else;
	} else;
	return( false );
}

/* ----------------------------------------------------------------------------------
	斜め脱出型135度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn135out( uint8_t i )
{
	if( (path[i].straight == 0) && (path[i+1].straight == 0) && (path[i+2].straight > 0) ) {
		if( (path[i].type == TURN_90) && (path[i+1].type == TURN_90) ) {
				return( path[i].direction == path[i+1].direction );
		} else;
	} else;
	return( false );
}

/* ----------------------------------------------------------------------------------
	90度ターンのパス判定
-----------------------------------------------------------------------------------*/
int8_t Path_IsTurn90v( uint8_t i )
{
	if( (path[i].straight == 0) && (path[i+1].straight == 0) && (path[i+2].straight == 0) ) {
		if( (path[i].type == TURN_90) && (path[i+1].type == TURN_90) ) {
			return( path[i].direction == path[i+1].direction );
		} else;
	} else;
	return( false );
}

/* ----------------------------------------------------------------------------------
	90度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn90v( uint8_t i )
{
	uint8_t j;

	path[i].type = TURN_90V;
	for( j = i + 1; j < NUM_PATH-1; j++ ) {
		if( path[j].type == GOAL ) {
			break;
		} else;

		path[j].straight = path[j+1].straight;
		path[j].direction = path[j+1].direction;
		path[j].type = path[j+1].type;
	}
}

/* ----------------------------------------------------------------------------------
	斜め侵入型45度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn45in( uint8_t i )
{
	path[i].straight--;
	path[i].type = TURN_45IN;
}

/* ----------------------------------------------------------------------------------
	斜め脱出型45度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn45out( uint8_t i )
{
	path[i+1].straight--;
	path[i].type = TURN_45OUT;
}

/* ----------------------------------------------------------------------------------
	斜め侵入型135度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn135in( uint8_t i )
{
	uint8_t j;

	path[i].straight--;
	path[i].type = TURN_135IN;
	for( j = i + 1; j < NUM_PATH-1; j++ ) {
		if( path[j].type == GOAL ) {
			break;
		} else;

		path[j].straight = path[j+1].straight;
		path[j].direction = path[j+1].direction;
		path[j].type = path[j+1].type;
	}
}

/* ----------------------------------------------------------------------------------
	斜め脱出型135度ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertTurn135out( uint8_t i )
{
	uint8_t j;

	path[i+2].straight--;
	path[i].type = TURN_135OUT;
	for( j = i + 1; j < NUM_PATH-1; j++ ) {
		if( path[j].type == GOAL ) {
			break;
		} else;

		path[j].straight = path[j+1].straight;
		path[j].direction = path[j+1].direction;
		path[j].type = path[j+1].type;
	}
}

/* ----------------------------------------------------------------------------------
	斜めのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertDiagonal( void )
{
	volatile uint8_t i, j, k;

	for( i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == GOAL ) {
			break;
		} else;

		if( (Path_IsTurn45in(i) == true) || (Path_IsTurn135in(i) == true) ) {
			for( j = i; j < NUM_PATH-1; j++ ) {
				if( path[j].type == GOAL ) {
					break;
				} else;
				if( (Path_IsTurn45out(j) == true) || (Path_IsTurn135out(j) == true) ) break;
			}
			if( path[j].type != GOAL ) {
				if( Path_IsTurn45in(i) == true )	Path_ConvertTurn45in(i);
				if( Path_IsTurn45out(j) == true )	Path_ConvertTurn45out(j);
				if( Path_IsTurn135out(j) == true )	Path_ConvertTurn135out(j);
				if( Path_IsTurn135in(i) == true )	Path_ConvertTurn135in(i);
				for( k = i + 1; k < j; k++ ) {
					if(Path_IsTurn90v(k) == true ) {
						Path_ConvertTurn90v(k);
						j--;
					} else;
				}
			} else;
		} else;
	}

	for( i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == GOAL ) {
			num = i + 1;
			break;
		} else;

		if( path[i].type >= TURN_45IN ) {
			i++;
			for( j = i; path[j].type < TURN_45OUT; j++ ) {
				if( path[j].type == TURN_90V ) {
					path[j].straight = j - i;
					for( k = i; k < NUM_PATH-1; k++ ) {
						if( path[k].type == GOAL ) {
							break;
						} else;
						path[k].straight	= path[k+j-i].straight;
						path[k].direction	= path[k+j-i].direction;
						path[k].type	 	= path[k+j-i].type;
					}
					j = i;
					i++;
				} else;
			}
			path[j].straight = j - i;
			for( k = i; k < NUM_PATH-1; k++ ) {
				if( path[k].type == GOAL ) {
					break;
				} else;
				path[k].straight	= path[k+j-i].straight;
				path[k].direction	= path[k+j-i].direction;
				path[k].type	 	= path[k+j-i].type;
			}
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	拡張ターンのパス変換
-----------------------------------------------------------------------------------*/
void Path_ConvertAdvancedTurn( void )
{
	for( int16_t i = 0; i < NUM_PATH-1; i++ ) {
		if( path[i].type == GOAL ) {
			num = i + 1;
			break;
		} else;

		if( path[i].type == TURN_45OUT && path[i+1].straight == 0 && path[i+1].type == TURN_45IN ) {
			if( path[i].direction == path[i+1].direction ) {
				path[i].type = TURN_KOJIMA;
				for( int16_t j = i + 1; j < NUM_PATH-1; j++ ) {
					if( path[j].type == GOAL ) {
						break;
					} else;

					path[j].straight = path[j+1].straight;
					path[j].direction = path[j+1].direction;
					path[j].type = path[j+1].type;
				}
			} else;
		} else;
	}
}

/* ----------------------------------------------------------------------------------
	パスの表示
-----------------------------------------------------------------------------------*/
void Path_Display( t_path temp_path )
{
	// 直線区間の表示
	if( temp_path.straight != 0 ) {
		if( temp_path.type >= TURN_90V && temp_path.type <= TURN_135OUT ) {
			printf("Diagonal\t : %d\n\r", temp_path.straight);
		} else {
			printf("Straight\t : %d\n\r", temp_path.straight);
		}
	}

	// ターンの表示
	switch (temp_path.type) {
		case TURN_0:		printf("Turn_0");		break;
		case TURN_90:		printf("Turn_90");		break;
		case TURN_90L:		printf("Turn_Large");	break;
		case TURN_180:		printf("Turn_180");		break;
		case TURN_90V:		printf("Turn_90v");		break;
		case TURN_45IN:		printf("Turn_45in");	break;
		case TURN_135IN:	printf("Turn_135in");	break;
		case TURN_45OUT:	printf("Turn_45out");	break;
		case TURN_135OUT:	printf("Turn_135out");	break;
		case TURN_KOJIMA:	printf("Turn_Kojima");	break;
		case GOAL:			printf("Goal!");		break;
		default:			printf("ERROR!\n\r");	break;
	}

	if( temp_path.type == GOAL ) {
		// 何もしない
	} else {
		if( temp_path.direction == RIGHT ) 	 	printf("_RIGHT");
		else if( temp_path.direction == LEFT )	printf("_LEFT");
		else								 	printf("_ERROR !!");
	}
}


void Path_DisplayAll( int8_t is_return )
{
	t_path temp_path;

	printf("\n\rStart!\n\r");
	for( uint8_t i = 0; i < NUM_PATH-1; i++ ) {
		if(is_return) {
			temp_path = Path_GetReturnSequence(i);
		} else {
			temp_path = Path_GetSequence(i);
		}

		Path_Display(temp_path);
		printf("\n\r");
		if( temp_path.type == GOAL ) {
			break;
		} else;
	}
	printf("\n\r");
}
