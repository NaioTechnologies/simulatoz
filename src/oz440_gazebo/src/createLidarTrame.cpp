#include "createLidarTrame.hpp"

#include <sys/ioctl.h>
#include <string.h>
#include <time.h>
//distances en mm.
void createTrame(int dist[271] , int albedo[271], char trame[10000],uint64_t nbMesures,uint64_t nbTelegrammes,struct timespec timeInit){
	char buffer[20];
	
	memset(trame,'\0',2000);
	
	strcat(trame,"sRA ");//commandType
	
	strcat(trame,"LMDscandata ");//command
	
	strcat(trame,"1 ");//versionNumber
	
	strcat(trame,"1 ");//deviceNumber
	
	strcat(trame,"000000 ");//SerialNumber
	
	strcat(trame,"0 0 ");//status 0 0->OK 0 1 -> error 0 2 -> pollutionWarning 0 4 pollutionError 
	
	sprintf(buffer,"%x ",(unsigned int)nbTelegrammes);
	strcat(trame,buffer);//TelegramCounter
	
	sprintf(buffer,"%x ",(unsigned int)nbMesures);
	strcat(trame,buffer);//ScanCounter
	
	sprintf(buffer,"%x ",(uint32_t)elapsedMillis(timeInit));
	strcat(trame,buffer);//timeSinceStartInμsec
	
	sprintf(buffer,"%x ",(uint32_t)elapsedMillis(timeInit));
	strcat(trame,buffer);//timeInμsec
	
	strcat(trame,"0 0 ");//Status of digsitalInputs ??
	
	strcat(trame,"0 0 ");//Status of digsitalOutputs ??
	
	strcat(trame,"0 ");//Reserved
	
	sprintf(buffer,"%x ",1500);
	strcat(trame,buffer);//Scan Frequency (15Hz)
	
	sprintf(buffer,"%x ",1500);
	strcat(trame,buffer);//Measurement Frequency ??
	
	strcat(trame,"0 ");//No encoders
	
	//Encoder position && speed not present because no encoders.
	
	strcat(trame,"1 ");//Amount of 16bits channels
	
	strcat(trame,"DIST1 ");//Type de message : distances
	
	strcat(trame,"3F800000 ");//scaleFactor
	
	strcat(trame,"00000000 ");//offset
	
	sprintf(buffer,"%x ",-135);
	strcat(trame,buffer);//startAngle
	
	sprintf(buffer,"%x ",5000);
	strcat(trame,buffer);//steps ???
	
	sprintf(buffer,"%x ",271);
	strcat(trame,buffer);//amountofData
	
	for (int i = 0;i<271;i++){
		int locDist = 0;
		if (dist[i] > 3999 || dist[i] < 20){
			locDist = 0;
		} else {
			locDist = dist[i];
		}
		sprintf(buffer,"%x ",locDist);
		strcat(trame,buffer);//distances
	}
	
	strcat(trame,"1 ");//Amount of bits channels
	
	strcat(trame,"RSSI1 ");//Type de message : intensités lumineuses
	
	strcat(trame,"3F800000 ");//scaleFactor
	
	strcat(trame,"00000000 ");//offset
	
	sprintf(buffer,"%x ",-135);
	strcat(trame,buffer);//startAngle
	
	sprintf(buffer,"%x ",5000);
	strcat(trame,buffer);//steps ???
	
	sprintf(buffer,"%x ",271);
	strcat(trame,buffer);//amountofData
	
	for (int i = 0;i<271;i++){
		int locLum = 0;
		if (dist[i] > 3999 || dist[i] < 20){
			locLum = 0;
		} else {
			locLum = 100;
		}
		sprintf(buffer,"%x ",locLum);
		strcat(trame,buffer);//distances
	}
	
	strcat(trame,"0 ");//Position data : 0 -> no position data
	
	strcat(trame,"0 ");//Device name : 0 -> no name
	
	strcat(trame,"0 ");//Device comment : 0 -> no comment
	
	strcat(trame,"0 ");//Device time : 0 -> no time
	
	strcat(trame,"0 ");//Event info : 0 -> no Event
	
	
}

long elapsedMillis(struct timespec dateDepart){
	struct timespec NOW;
	long ecart;
	
	clock_gettime(CLOCK_MONOTONIC_RAW, &NOW);
	
	ecart = (((long)(NOW.tv_sec) - (long)(dateDepart.tv_sec)) * 1000L) + ((long)(NOW.tv_nsec) - (long)(dateDepart.tv_nsec))/1000000L;
	return ecart;
}
