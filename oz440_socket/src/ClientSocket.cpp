// Implementation of the ClientSocket class

#include "../include/oz440_socket/ClientSocket.h"
#include "../include/oz440_socket/SocketException.h"


ClientSocket::ClientSocket ( std::string host, int port )
{
  if ( ! Socket::create() )
    {
      throw SocketException ( "Could not create client socket." );
    }

  if ( ! Socket::connect ( host, port ) )
    {
      throw SocketException ( "Could not bind to port." );
    }

}


void ClientSocket::sendToSock( const uint8_t* data, const int size)
{
  if ( ! Socket::send ( data , size ) )
    {
      throw SocketException ( "Could not write to socket." );
    }
}


int ClientSocket::recvFromSock ( uint8_t* data )
{
  int size = Socket::recv ( data );
  if ( ! size )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return size;
}
