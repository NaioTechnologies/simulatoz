
#include "DriverSocket.hpp"

#include "Lidar.h"

using namespace std::chrono;
using namespace std::chrono_literals;

//*****************************************  --  CONSTRUCTOR / DESTRUCTOR  --  *****************************************

Lidar::Lidar(int server_port)
        : stop_asked_{ false }
        , connect_thread_started_ { false }
        , connect_thread_ { }
        , socket_connected_ { false }
        , server_port_ {server_port}
        , server_socket_desc_ { -1 }
        , socket_desc_ { -1 }
        , socket_access_ {}
        , nbMesures_ { 1 }
        , nbTelegrammes_ { 1 }
{
    init();
}

Lidar::~Lidar()
{
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  INIT  --  *************************************************************

void Lidar::init()
{
    connect_thread_ = std::thread( &Lidar::connect, this );
    connect_thread_.detach();

    read_thread_ = std::thread( &Lidar::read_thread, this );
    read_thread_.detach();
}

//*****************************************  --  SUBSCRIBE  --  *******************************************************

void Lidar::subscribe( ros::NodeHandle& node )
{
    lidar_sub_ = node.subscribe( "/oz440/laser/scan", 50, &Lidar::callback_lidar, this );
}

//*****************************************  --  ASK STOP  --  *********************************************************

void Lidar::cleanup()
{
    stop_asked_ = true;
    disconnect();
    close(server_socket_desc_);
}

//*****************************************  --  CONNECT  --  **********************************************************

void Lidar::connect(){

    connect_thread_started_ = true;

    server_socket_desc_ = DriverSocket::openSocketServer( (uint16_t) server_port_ );

    while( !stop_asked_ )
    {
        if( !socket_connected_ and server_socket_desc_ > 0 )
        {
            socket_desc_ = DriverSocket::waitConnectTimer( server_socket_desc_, stop_asked_ );

            if ( socket_desc_ > 0 )
            {
                socket_connected_ = true;

                ROS_ERROR( "OzCore Lidar Socket Connected" );
            }
        }
        else{
            std::this_thread::sleep_for(500ms);
        }
    }

    connect_thread_started_ = false;
}

//*****************************************  --  READ THREAD  --  ******************************************************

void Lidar::read_thread(){

    char received_buffer[4096];


    while ( !stop_asked_ )
    {
        if (socket_connected_)
        {
            memset( received_buffer, '\0', 1000 );

            socket_access_.lock();
            ssize_t size = read( socket_desc_, received_buffer, 4096 );
            socket_access_.unlock();

            if (size > 0)
            {
                received_buffer[ size ] = '\0';

                if (strncmp("\x02sRN LMDscandata 1\x03", (char *) received_buffer, strlen("\x02sRN LMDscandata 1\x03")) == 0)
                {
//                    send_packet();
                }
            }
            else
            {
                if( errno == 32 or errno == 104 or ( size == 0 and errno == 11 )){
                    disconnect();
                }
            }

            std::this_thread::sleep_for(1ms);
        }
        else
        {
            std::this_thread::sleep_for(100ms);
        }
    }

}

//*****************************************  --  DISCONNECT  --  *******************************************************

void Lidar::disconnect(){

    close( socket_desc_ );

    socket_connected_ = false;

    ROS_ERROR("OzCore Lidar Socket Disconnected");
}

// *********************************************************************************************************************

void Lidar::callback_lidar( const sensor_msgs::LaserScan::ConstPtr& lidar_msg )
{
    if( socket_connected_ ) {
        char trame[10000];

        int lidar[271];
        int albedo[271];

        struct timespec timeInit;
        clock_gettime(CLOCK_MONOTONIC_RAW, &timeInit);

        for (int i = 0; i < 271; ++i) {

            if (i >= 45 and i < 226) {
                lidar[i] = (uint16_t) (lidar_msg->ranges[270 - i] * 1000); //Convert meters to millimeters
            } else {
                lidar[i] = 0;
            }
            albedo[i] = 0;
        }

        nbMesures_++;
        nbTelegrammes_++;

        createTrame(lidar, albedo, trame, nbMesures_, nbTelegrammes_, timeInit);

        socket_access_.lock();
        ssize_t write_size = write(socket_desc_, trame, strlen(trame));
        socket_access_.unlock();

        if (write_size != strlen(trame)) {
            ROS_ERROR("Error sending Lidar trame");
        } else {
            ROS_INFO("Lidar packet send");
        }
    }
}

void Lidar::createTrame(int dist[271] , int albedo[271], char trame[10000],uint64_t nbMesures,uint64_t nbTelegrammes,struct timespec timeInit){
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

long Lidar::elapsedMillis(struct timespec dateDepart){
    struct timespec NOW;
    long ecart;

    clock_gettime(CLOCK_MONOTONIC_RAW, &NOW);

    ecart = (((long)(NOW.tv_sec) - (long)(dateDepart.tv_sec)) * 1000L) + ((long)(NOW.tv_nsec) - (long)(dateDepart.tv_nsec))/1000000L;
    return ecart;
}
