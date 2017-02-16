#ifndef CREATE_LIDAR_TRAME_H
#define CREATE_LIDAR_TRAME_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>

//Distance en mm
void createTrame( uint16_t dist[271], uint8_t albedo[271], char trame[1024], uint64_t nbMesures,
				  uint64_t nbTelegrammes, struct timespec timeInit );

long elapsedMillis(struct timespec dateDepart);


#endif
