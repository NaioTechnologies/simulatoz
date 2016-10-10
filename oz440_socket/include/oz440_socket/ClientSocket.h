// Definition of the ClientSocket class

#ifndef ClientSocket_class
#define ClientSocket_class

#include "Socket.h"


class ClientSocket : private Socket
{
 public:

  ClientSocket ( std::string host, int port );
  virtual ~ClientSocket(){};

  void sendToSock( const uint8_t* data, const int size);
  int recvFromSock ( uint8_t* data );

};


#endif
