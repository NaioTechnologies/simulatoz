#ifndef CREATE_LIDAR_TRAME_H
#define CREATE_LIDAR_TRAME_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>

//Distance en mm
void createTrame(int dist[271] , int albedo[271], char trame[10000],uint64_t nbMesures,uint64_t nbTelegrammes,struct timespec timeInit);
long elapsedMillis(struct timespec dateDepart);


#endif
