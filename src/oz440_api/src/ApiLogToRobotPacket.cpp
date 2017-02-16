#include "ApiLogToRobotPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiLogToRobotPacket::ApiLogToRobotPacket( )
{

}

//=============================================================================
//
ApiLogToRobotPacket::ApiLogToRobotPacket( std::string message_  )
	: 	message( message_ )
{

}

//=============================================================================
//
ApiLogToRobotPacket::~ApiLogToRobotPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiLogToRobotPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 127 );

	for( uint i = 0; i < 127 ; i++ )
	{
		if( i < message.length() )
		{
			(*buffer)[cpt++] = static_cast<uint8_t>( message[ i ] );
		}
		else
		{
			(*buffer)[cpt++] = static_cast<uint8_t>( '\0' );
		}
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiLogToRobotPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	bool stop1 = false;
	for( uint i = 0; i < 127 ; i++ )
	{
		char c = static_cast<char>( buffer[ cpt++ ] );

		if( c == '\0' )
			stop1 = true;

		if( !stop1 )
			message += c;
	}
}
