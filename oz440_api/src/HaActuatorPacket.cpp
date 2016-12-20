#include "../include/oz440_api/HaActuatorPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"
#include "/usr/include/stdint.h"

//=============================================================================
//
HaActuatorPacket::HaActuatorPacket( )
{

}

//=============================================================================
//
HaActuatorPacket::HaActuatorPacket( bool isRequest_, bool isPosition_, uint8_t position_ )
	: 	isRequest{ isRequest_ },
		isPosition{ isPosition_ },
		position{ position_ }
{

}

//=============================================================================
//
HaActuatorPacket::~HaActuatorPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr HaActuatorPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 + 1 + 1 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( isRequest );
	(*buffer)[cpt++] = static_cast<uint8_t>( isPosition );
	(*buffer)[cpt++] = static_cast<uint8_t>( position );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaActuatorPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	isRequest = static_cast<bool>( buffer[cpt++] );
	isPosition = static_cast<bool>( buffer[cpt++] );
	position = static_cast<uint8_t>( buffer[cpt++] );
}

