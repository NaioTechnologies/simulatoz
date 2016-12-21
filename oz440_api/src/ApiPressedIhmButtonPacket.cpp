#include "ApiPressedIhmButtonPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiPressedIhmButtonPacket::ApiPressedIhmButtonPacket( )
{

}

//=============================================================================
//
ApiPressedIhmButtonPacket::ApiPressedIhmButtonPacket( PressedIhmButtonIndex pressedIhmButton_ )
	: 	pressedIhmButton{ pressedIhmButton_ }
{

}

//=============================================================================
//
ApiPressedIhmButtonPacket::~ApiPressedIhmButtonPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiPressedIhmButtonPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( pressedIhmButton );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiPressedIhmButtonPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	pressedIhmButton = static_cast< PressedIhmButtonIndex >( buffer[ cpt++ ] );
}

