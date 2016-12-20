#include "../include/oz440_api/ApiSmsPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
ApiSmsPacket::ApiSmsPacket( )
{

}

//=============================================================================
//
ApiSmsPacket::ApiSmsPacket( SmsType smsType_, std::string recipient_, std::string message_  )
	: 	smsType( smsType_ ),
		recipient( recipient_ ),
		message( message_ )
{

}

//=============================================================================
//
ApiSmsPacket::~ApiSmsPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiSmsPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 + 20 + 200 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( smsType );

	for( uint i = 0; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( recipient[ i ] );
	}

	for( uint i = 0; i < 200 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( message[ i ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiSmsPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	smsType = static_cast<SmsType>( buffer[ cpt++ ] );

	bool stop1 = false;
	for( uint i = 0; i < 20 ; i++ )
	{
		char c = static_cast<char>( buffer[ cpt++ ] );

		if( c == '\0' )
			stop1 = true;

		if( !stop1 )
			recipient += c;
	}

	bool stop2 = false;
	for( uint i = 0; i < 200 ; i++ )
	{
		char c = static_cast<char>( buffer[ cpt++ ] );

		if( c == '\0' )
			stop2 = true;

		if( !stop2 )
			message += c;
	}
}
