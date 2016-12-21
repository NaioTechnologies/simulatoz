#include "HaActuatorPacket.hpp"
#include "vitals/CLByteConversion.h"

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
cl_copy::BufferUPtr HaActuatorPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 1 + 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( isRequest );
	(*buffer)[cpt++] = static_cast<uint8_t>( isPosition );
	(*buffer)[cpt++] = static_cast<uint8_t>( position );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaActuatorPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	isRequest = static_cast<bool>( buffer[cpt++] );
	isPosition = static_cast<bool>( buffer[cpt++] );
	position = static_cast<uint8_t>( buffer[cpt++] );
}

