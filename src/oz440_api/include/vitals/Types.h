//=============================================================================
//
//  Copyright (C)  2014  Naio Technologies
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//=============================================================================

#ifndef _TYPES_H_
#define _TYPES_H_

#include <time.h>
#include <pthread.h>
#include <inttypes.h>

#ifndef UNUSED
#ifdef __GNUC__
#define UNUSED __attribute__ ((unused))
#else
#define UNUSED
#endif
#endif

//#include "Helpers/Chrono.h"



typedef enum _cote {
	C_CENTRE = 0,
	C_DROITE = 1,
	C_GAUCHE = -1
}COTE;

typedef enum _LineMode {
	LM_LIDAR = 0,
	LM_CULTURE_BASSE = 1,
	LM_BACHE = 2,
	LM_TUYAU = 3,
}LINE_MODE;


enum class API_MESSAGE : uint8_t
{
	API_MESSAGE_NONE = 0x00,

	API_MESSAGE_PAUSE_IHM = 0x01,

	API_MESSAGE_SMS_PAUSE = 0x02,

	API_MESSAGE_BATTERY_TOO_LOW_FOR_AUTO_ERROR = 0x03,

	API_MESSAGE_PAUSE_REMOTE = 0x04,

	API_MESSAGE_CANNOT_DETECT_END_OF_ROW_ERROR = 0x05,

	API_MESSAGE_INVALID_MAP_ERROR = 0x06,

	API_MESSAGE_TERMINATED_WORK = 0x07,

	API_MESSAGE_LIDAR_ISSUE_ERROR = 0x08,

	API_MESSAGE_ROBOT_UPSIDEDOWN_ERROR = 0x09,

	API_MESSAGE_TERMINATED_ROW = 0x0A,

	API_MESSAGE_OBSTACLE_DETECTED_ERROR = 0x0B,

	API_MESSAGE_ACTUATOR_STUCK_ERROR = 0x0C,

	API_MESSAGE_LOST_IN_ROW_ERROR = 0x0D,

	API_MESSAGE_CHANGE_LINE_ERROR = 0x0E,

	API_MESSAGE_CHANGE_LINE_GYRO_ISSUE_ERROR = 0x0F,

	API_MESSAGE_NOT_IN_FRONT_OF_ROW_ERROR = 0x10,

	API_MESSAGE_TOO_DARK_FOR_CAMERA = 0x11,
};



typedef enum _erreur {
	ERR_NONE = 0,
	ERR_TROU_CULTURE = 1,
	ERR_FIN_NON_TROUVEE = 2,
	ERR_FIN = 3,
	ERR_OBSTACLE = 4,
	ERR_MOUSTACHE = 5,
	ERR_ROUE_BLOQUEE = 6,
	ERR_DEMI_TOUR_RATE = 7,
	ERR_LIDAR = 8,
	ERR_PATINAGE = 9,
	ERR_GYRO = 10,
	ERR_ANCRAGE = 11,
	ERR_ANCRAGE_FAIBLE = 12,
	ERR_FIN_PIQUETS = 13,
} ERREUR;

typedef enum _manoeuvre {
	MAN_NONE = 0,
	MAN_DEG_OUTIL = 1,
	MAN_MOUSTACHE = 2,
	MAN_DEBOURRAGE = 3,
	MAN_DEBOUR_LAT = 4,
	MAN_OBSTACLE = 5,
} MANOEUVRE;

typedef enum _batterie_type {
	B_NONE = 0,
	B_PLOMB = 1,
	B_LITHIUM = 2,
} BATTERIE_TYPE;

typedef enum _lang {
	L_FR = 0,
	L_EN = 1,
	L_ES = 2,
} LANG;

typedef struct point_ {
	double x;
	double y;
} point;

//Fonctions de comparaison pour les tris
inline static int comparePointX ( const void * a , const void * b ){
	if ( ((point*)a)->x <  ((point*)b)->x ) return -1;
	if ( ((point*)a)->x >  ((point*)b)->x ) return 1;
	if ( ((point*)a)->y <  ((point*)b)->y ) return -1;
	if ( ((point*)a)->y >  ((point*)b)->y ) return 1;
	return 0;
}
inline static int comparePointY ( const void * a , const void * b ){
	if ( ((point*)a)->y <  ((point*)b)->y ) return -1;
	if ( ((point*)a)->y >  ((point*)b)->y ) return 1;
	if ( ((point*)a)->x <  ((point*)b)->x ) return -1;
	if ( ((point*)a)->x >  ((point*)b)->x ) return 1;
	return 0;
}


typedef enum _special {
	SPE_NO = 0,
	SPE_LINARD = 1,
	SPE_RTK = 2,
	SPE_CIVC = 3,
} SPECIAL;

#ifndef T_MESURE
#define T_MESURE
typedef struct _mesure_{
	int dist;
	unsigned int intensity;
	double angle;
}mesure;
#endif

#ifndef T_LASER_SCAN
#define T_LASER_SCAN
typedef struct _LaserScan_{
	uint64_t scan_time;
	int nbMesures;
	double angle_min;
	double angle_max;
	double angle_increment;
	mesure mesures[1000];
} LaserScan;
#endif

//categories
//

typedef struct _om {
	int md;//m1
	int mg;//m2
} ORDRE_MOTEUR ;

typedef struct _Consigne {
	double speed = 0;
	double ecartLat = 0;
	double ecartAng = 0;//Non utilisé pour le moment
	double urgence = 1;//Pas d'urgence. (va jusqu'à 5, pour une urgence forte)
	int klaxonFaible = 0;
	int klaxonFort = 0;
	COTE side = C_CENTRE;
} Consigne;


typedef struct _parcelle_ {
	char nom[100];
	int nbRangees = 1;
	double largRangee[100];
	int longAllee[100];
	double distDemiTour[100];
	int tailleCult = 0;
	double ecartBord = 0.1;
	COTE sensDT = C_DROITE;
	int nbPass = 1;
} parcelle;


#endif
