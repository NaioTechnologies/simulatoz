#include "ApiSmsPacket.hpp"
#include "vitals/CLByteConversion.h"

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
cl_copy::BufferUPtr ApiSmsPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 20 + 200 );

	(*buffer)[cpt++] = static_cast<uint8_t>( smsType );

	for( uint i = 0; i < 20 ; i++ )
	{
		if( i < recipient.length() )
		{
			(*buffer)[cpt++] = static_cast<uint8_t>( recipient[i] );
		}
		else
		{
			(*buffer)[cpt++] = static_cast<uint8_t>( '\0' );
		}
	}

	for( uint i = 0; i < 200 ; i++ )
	{
		if( i < message.length() )
		{
			(*buffer)[cpt++] = static_cast<uint8_t>( message[i] );
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
void ApiSmsPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

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
