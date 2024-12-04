#include "motion.h"
#include "application.h"


void Route_DebugData( void )
{
	uint8_t debug[] = {
		0, 15, 1, 15, 2, 15, 3, 15, 4, 15, 5, 15, 6, 15, 7, 15, 8, 15, 9, 15, 10, 15, 11, 15
	};
	Route_SetPath(debug, sizeof(debug));
}

void Route_SetPath( uint8_t *data, uint8_t length )
{
	int8_t 		next_direction = -1;

	// パスのリセット
	Path_Reset();

	// 初期位置を設定
	t_position 	my = {data[0], data[1], -1};
	if( my.x == 0 && my.y == 0 ) {
		my.dir = NORTH;
	} else if( my.x == 0 && my.y == MAZE_SIZE-1 ) {
		my.dir = EAST;
	} else if( my.x == MAZE_SIZE-1 && my.y == MAZE_SIZE-1 ) {
		my.dir = SOUTH;
	} else if( my.x == MAZE_SIZE-1 && my.y == 0 ) {
		my.dir = WEST;
	} else {
		return;
	}
	Position_SetMyPlace(my);

	Vehicle_SetGlobalX( (2.f * my.x + 1.f) * SECTION_SIZE + 15.f );
	Vehicle_SetGlobalY( (2.f * my.y + 1.f) * SECTION_SIZE + 15.f );
	Vehicle_SetYaw( my.dir * PI / 2.f );

	// 受信した経路からパスに変換
	for( uint8_t i = 2; i < length; i += 2 ) {
		if( my.x + 1 == data[i] ) {
			switch( my.dir ) {
				case EAST:	next_direction = FRONT;	break;
				case NORTH:	next_direction = RIGHT;	break;
				case WEST:	next_direction = REAR;	break;
				case SOUTH:	next_direction = LEFT;	break;
			}
		} else if( my.y + 1 == data[i + 1] ) {
			switch( my.dir ) {
				case EAST:	next_direction = LEFT;	break;
				case NORTH:	next_direction = FRONT;	break;
				case WEST:	next_direction = RIGHT;	break;
				case SOUTH:	next_direction = REAR;	break;
			}
		} else if( my.x - 1 == data[i] ) {
			switch( my.dir ) {
				case EAST:	next_direction = REAR;	break;
				case NORTH:	next_direction = LEFT;	break;
				case WEST:	next_direction = FRONT;	break;
				case SOUTH:	next_direction = RIGHT;	break;
			}
		} else if( my.y - 1 == data[i + 1] ) {
			switch( my.dir ) {
				case EAST:	next_direction = RIGHT;	break;
				case NORTH:	next_direction = REAR;	break;
				case WEST:	next_direction = LEFT;	break;
				case SOUTH:	next_direction = FRONT;	break;
			}
		}
		Position_MoveMyPlace( next_direction );
		// スタート区画は半区画直進またはその場旋回
		if( i == 2 ) {
			switch( next_direction ) {
				case FRONT:	Path_SetStraightSection(1);				break;
				case RIGHT:
					Path_SetTurnSection(TURN_0, RIGHT);
					Path_SetStraightSection(1);
				break;
				case LEFT:
					Path_SetTurnSection(TURN_0, LEFT);
					Path_SetStraightSection(1);
				break;
			}
		} else {
			switch( next_direction ) {
				case FRONT:	Path_SetStraightSection(2);				break;
				case RIGHT:	Path_SetTurnSection(TURN_90, RIGHT);	break;
				case LEFT:	Path_SetTurnSection(TURN_90, LEFT);		break;
				case REAR:	Path_SetTurnSection(TURN_180, REAR);	break;
			}
		}
		my = Position_GetMyPlace();
//		printf("N: %02d, NxetDir: %02d, (X: %02d, Y: %02d, Dir: %01d)\r\n",
//				i / 2, next_direction, my.x, my.y, my.dir);
	}
	Path_SetStraightSection(1);
	Path_SetTurnSection(GOAL, -1);
//	Path_DisplayAll( false );

	// 掃除用の超信地旋回のパスへ変換
	Path_ConvertCleaner();
//	Path_DisplayAll( false );
}
