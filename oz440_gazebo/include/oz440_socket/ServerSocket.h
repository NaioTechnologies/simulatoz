// Definition of the ServerSocket class

#ifndef ServerSocket_class
#define ServerSocket_class

#include "Socket.h"


class ServerSocket : private Socket
{
 public:

  ServerSocket ( int port );
  ServerSocket (){};
  virtual ~ServerSocket();

  void sendToSock( const uint8_t* data, const int size);
  int recvFromSock ( uint8_t* data );

  void set_no_blocking ( const bool b);
  void accept ( ServerSocket& );

};


#endif
