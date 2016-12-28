#ifndef MY_SOCKET_H
#define MY_SOCKET_H

#include <sys/ioctl.h>

#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket(s) close(s)
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;


class DriverSocket{
public:
    DriverSocket();
    ~DriverSocket();

    static void error(const char *msg);
    //Initialisation du socket
    static SOCKET openSocketClient(const char* address, uint16_t port);


    //Ouvre le port donné
    static SOCKET openSocketServer(uint16_t portNum);

    //Attend qu'un joueur se connecte
    static SOCKET waitConnect(SOCKET sockfd);
    static SOCKET waitConnectTimer(SOCKET sockfd);


    //Lit une entrée joueur. Retourne 1 si lecture OK, 0 si rien
    static int readNonBlockSocket(SOCKET fd, char buff[], size_t size);

    static void sendToSocket(SOCKET socket, char message[]);
};
#endif