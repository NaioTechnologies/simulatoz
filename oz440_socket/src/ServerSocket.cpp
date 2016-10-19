// Implementation of the ServerSocket class

#include "../include/oz440_socket/ServerSocket.h"
#include "../include/oz440_socket/SocketException.h"


ServerSocket::ServerSocket ( int port )
{
  if ( ! Socket::create() )
    {
      throw SocketException ( "Could not create server socket." );
    }

  if ( ! Socket::bind ( port ) )
    {
      throw SocketException ( "Could not bind to port." );
    }

  if ( ! Socket::listen() )
    {
      throw SocketException ( "Could not listen to socket." );
    }

}

ServerSocket::~ServerSocket()
{
}

void ServerSocket::sendToSock( const uint8_t* data, const int size)
{
  if ( ! Socket::send ( data , size ) )
    {
      throw SocketException ( "Could not write to socket." );
    }
}


int ServerSocket::recvFromSock ( uint8_t* data )
{
  int size = Socket::recv ( data );
  if ( ! size )
    {
      throw SocketException ( "Could not read from socket." );
    }

  return size;
}

void ServerSocket::set_no_blocking ( const bool b)
{
  Socket::set_non_blocking ( b );
}

void ServerSocket::accept ( ServerSocket& sock )
{
  if ( ! Socket::accept ( sock ) )
    {
      throw SocketException ( "Could not accept socket." );
    }
}
