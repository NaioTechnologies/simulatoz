
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>

#include "DriverSocket.hpp"


//Affichage des erreurs
void DriverSocket::error(const char *msg){
    perror(msg);
}

//Ouvre la socket vers le serveur
SOCKET DriverSocket::openSocketClient(const char* address, uint16_t port){
    SOCKET sockfd;
    SOCKADDR_IN serv_addr;
    //struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);//TCP
    //sockfd = socket(AF_INET, SOCK_DGRAM, 0);//UDP
    if (sockfd < 0) {
        fprintf(stderr,"ERROR opening socket");
        return -1;
    }

    /*server = gethostbyname(address);
    if (server == NULL) {
        p_logger->logLvl2("ERROR no such host");
        fprintf(stderr,"ERROR, no such host\n");
        close(sockfd);
        return -1;
    }*/

    memset((char *) &serv_addr,0, sizeof(serv_addr));

    serv_addr.sin_addr.s_addr = inet_addr(address);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    if (connect(sockfd,(SOCKADDR *) &serv_addr,sizeof(serv_addr)) < 0) {
        //fprintf(stderr,"Error connecting\n");
        return -1;
    }


    int flag = 1;
    int result = setsockopt(sockfd,            /* socket affected */
                            IPPROTO_TCP,     /* set option at TCP level */
                            TCP_NODELAY,     /* name of option */
                            (char *) &flag,  /* the cast is historical cruft */
                            sizeof(int));    /* length of option value */
    if (result < 0){
        fprintf(stderr,"error on No_Delay");
        return -1;
    }

    //Test de re-use
    flag = 1;
    result = setsockopt(sockfd,            /* socket affected */
                        SOL_SOCKET,     /* set option at TCP level */
                        SO_REUSEADDR,     /* name of option */
                        (char *) &flag,  /* the cast is historical cruft */
                        sizeof(int));    /* length of option value */
    if (result < 0){
        fprintf(stderr,"error on re-use");
        return -1;
    }

    return sockfd;
}





SOCKET DriverSocket::openSocketServer(uint16_t portNum){
    SOCKET sockfd;
    SOCKADDR_IN serv_addr;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);//TCP
    //sockfd = socket(AF_INET, SOCK_DGRAM, 0);//UDP
    if (sockfd < 0){
        error("ERROR opening socket");
    }
    memset((char *) &serv_addr, 0,sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portNum);
    if (bind(sockfd, (SOCKADDR *) &serv_addr, sizeof(serv_addr)) < 0){
        error("ERROR on binding");
    }
    int flag = 1;
    int result = setsockopt(sockfd,            /* socket affected */
                            IPPROTO_TCP,     /* set option at TCP level */
                            TCP_NODELAY,     /* name of option */
                            (char *) &flag,  /* the cast is historical cruft */
                            sizeof(int));    /* length of option value */
    if (result < 0){
        error("error on No_Delay");
    }
    flag = 1;
    result = setsockopt(sockfd,            /* socket affected */
                        SOL_SOCKET,     /* set option at TCP level */
                        SO_REUSEADDR,     /* name of option */
                        (char *) &flag,  /* the cast is historical cruft */
                        sizeof(int));    /* length of option value */
    if (result < 0){
        fprintf(stderr,"error on re-use");
        return -1;
    }

    fcntl(sockfd, F_SETFL, O_NONBLOCK);

    return sockfd;
}

//Attend qu'un client se connecte
SOCKET DriverSocket::waitConnect(SOCKET sockfd){
    socklen_t clilen;
    SOCKADDR_IN cli_addr;
    SOCKET newsockfd;

    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    do {
        newsockfd = accept(sockfd, (SOCKADDR *) &cli_addr, &clilen);
        usleep(1000);
    } while (newsockfd < 0);

    if (newsockfd < 0){
        error("ERROR on accept");
    }

    int flag = 1;
    int result = setsockopt(newsockfd,/* socket affected */
                            IPPROTO_TCP,     /* set option at TCP level */
                            TCP_NODELAY,     /* name of option */
                            (char *) &flag,  /* the cast is historical cruft */
                            sizeof(int));    /* length of option value */
    if (result < 0){
        error("error on No_Delay");
    }
    int flags = fcntl(newsockfd, F_GETFL, 0);
    fcntl(newsockfd, F_SETFL, flags | O_NONBLOCK);

    return newsockfd;

}



//Attend qu'un client se connecte
SOCKET DriverSocket::waitConnectTimer(SOCKET sockfd, std::atomic<bool>& terminate_ptr){
    socklen_t clilen;
    SOCKADDR_IN cli_addr;
    SOCKET newsockfd;

    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    do {
        newsockfd = accept(sockfd, (SOCKADDR *) &cli_addr, &clilen);
        usleep(1000);
    } while (newsockfd < 0 and !(terminate_ptr));

    if (newsockfd < 0){
        printf("ERROR on accept");
        return(-1);
    }

    int flag = 1;
    int result = setsockopt(newsockfd,/* socket affected */
                            IPPROTO_TCP,     /* set option at TCP level */
                            TCP_NODELAY,     /* name of option */
                            (char *) &flag,  /* the cast is historical cruft */
                            sizeof(int));    /* length of option value */
    if (result < 0) {
        error("error on No_Delay");
    }
    int flags = fcntl(newsockfd, F_GETFL, 0);
    fcntl(newsockfd, F_SETFL, flags | O_NONBLOCK);


    return newsockfd;

}

int DriverSocket::readNonBlockSocket(SOCKET fd, char buff[], size_t size){
    int n = 0;

    struct timeval timeout;
    fd_set readfs;

    timeout.tv_usec = 100;
    timeout.tv_sec = 0;

    memset(buff,'\0',size);

    FD_ZERO(&readfs);
    FD_SET(fd, &readfs);

    //Regarde ce qu'il y a en attente, avec un timeout de 100μs
    if(select(fd + 1, &readfs, NULL, NULL, &timeout) < 0){
        error("select()");
    }

    //Des choses à lire dans ce socket
    if( FD_ISSET ( fd, &readfs ) ) {
        n = (int)read(fd,buff,size-1);
    }

    return n;
}

void DriverSocket::sendToSocket(SOCKET socket, char message[]){
    int n2;

    n2 = (int)write(socket,message,strlen(message));
    if (n2 < 0){
        printf("erreur d'écriture\n");
    }

}
