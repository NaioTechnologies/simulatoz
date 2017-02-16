#include "ApiMoveActuatorPacket.hpp"
#include "vitals/CLByteConversion.h"

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
cl_copy::BufferUPtr ApiMoveActuatorPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( position );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiMoveActuatorPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	position = static_cast< uint8_t >( buffer[ cpt++ ] );
}

