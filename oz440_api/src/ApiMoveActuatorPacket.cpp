#include "../include/oz440_api/ApiMoveActuatorPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
ApiMoveActuatorPacket::ApiMoveActuatorPacket( )
{

}

//=============================================================================
//
ApiMoveActuatorPacket::ApiMoveActuatorPacket( uint8_t position_  )
	: 	position{ position_ }
{

}

//=============================================================================
//
ApiMoveActuatorPacket::~ApiMoveActuatorPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiMoveActuatorPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( position );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiMoveActuatorPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	position = static_cast< uint8_t >( buffer[ cpt++ ] );
}

