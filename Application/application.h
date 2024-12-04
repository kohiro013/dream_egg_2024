#pragma once

#include "module.h"
#include "motion.h"

#define GOAL_SIZE			(1)
#define MAZE_SIZE			(16)
#define SECTION_SIZE 		(45.f)

typedef struct {
	int8_t x;
	int8_t y;
	int8_t dir;
} t_position;

typedef struct {	// 走行パス用構造体
	volatile int8_t straight;
	volatile int8_t direction;
	volatile int8_t type;
} t_path;

typedef struct {	// 直進用構造体
	volatile float acceleration;
	volatile float deceleration;
	volatile float max_velocity;
} t_init_straight;

// 自己位置関数群(position.c)
t_position 	Position_GetMyPlace( void );
void 		Position_SetMyPlace( t_position );
void 		Position_Reset( void );
t_position	Position_RotateMyDirection( int8_t );
t_position	Position_MoveMyPlace( int8_t );
int8_t 		Position_GetIsGoal( int8_t, int8_t );

// パス関数群(path.c)
void 		Path_Reset( void );
void 		Path_SetStraightSection( int8_t );
void 		Path_SetTurnSection( int8_t, int8_t );
t_path 		Path_GetSequence( uint8_t );
t_path 		Path_GetReturnSequence( uint8_t );
uint8_t 	Path_GetSequenceNumber( void );
void 		Path_ConvertCleaner( void );
void 		Path_ConvertTurnLarge( void );
void 		Path_ConvertTurn180( void );
void 		Path_ConvertDiagonal( void );
void 		Path_ConvertAdvancedTurn( void );
void 		Path_Display( t_path );
void 		Path_DisplayAll( int8_t );

// 経路生成関連関数群(route.c)
void 		Route_DebugData( void );
void 		Route_SetPath( uint8_t*, uint8_t );


void 		Cleaner_Run( int8_t );
