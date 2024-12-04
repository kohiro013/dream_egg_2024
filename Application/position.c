#include "application.h"


volatile static t_position		my;

/* ----------------------------------------------------------------------------------
	自己位置を取得
-----------------------------------------------------------------------------------*/
t_position Position_GetMyPlace( void )
{
	return my;
}

/* ----------------------------------------------------------------------------------
	自己位置を入力
-----------------------------------------------------------------------------------*/
void Position_SetMyPlace( t_position pos )
{
	my = pos;
}

/* ----------------------------------------------------------------------------------
	自己位置をリセット
-----------------------------------------------------------------------------------*/
void Position_Reset( void )
{
	my.x = my.y = 0;
	my.dir = NORTH;
}


/* ----------------------------------------------------------------------------------
	自己位置を回転
-----------------------------------------------------------------------------------*/
t_position Position_RotateMyDirection( int8_t dir )
{
	my.dir += (dir - 1);
	if( my.dir < EAST ) {
		my.dir = SOUTH;
	} else if( my.dir > SOUTH ) {
		my.dir -= ( SOUTH + 1 );
	} else;
	return my;
}

/* ----------------------------------------------------------------------------------
	自己位置を移動
-----------------------------------------------------------------------------------*/
t_position Position_MoveMyPlace( int8_t dir )
{
	Position_RotateMyDirection( dir );
	my.x += (int)( arm_cos_f32(DEG2RAD(90.f * my.dir)) + (SIGN(arm_cos_f32(DEG2RAD(90.f * my.dir))) * 0.5f) );
	my.y += (int)( arm_sin_f32(DEG2RAD(90.f * my.dir)) + (SIGN(arm_sin_f32(DEG2RAD(90.f * my.dir))) * 0.5f) );
	return my;
}

/* ----------------------------------------------------------------------------------
	自己位置がゴール座標と一致したか
-----------------------------------------------------------------------------------*/
int8_t Position_GetIsGoal( int8_t gx, int8_t gy )
{
	if( gx == 0 && gy == 0 ) {
		if( my.x == 0 && my.y == 0 ) {
			return true;
		} else {
			return false;
		}
	} else {
		if( gx <= my.x && my.x < gx + GOAL_SIZE ) {
			if( gy <= my.y && my.y < gy + GOAL_SIZE ) {
				return true;
			} else;
		} else;
	}
	return false;
}
