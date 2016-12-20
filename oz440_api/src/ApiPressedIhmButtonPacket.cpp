#include "../include/oz440_api/ApiPressedIhmButtonPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

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
cl::BufferUPtr ApiPressedIhmButtonPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( pressedIhmButton );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiPressedIhmButtonPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	pressedIhmButton = static_cast< PressedIhmButtonIndex >( buffer[ cpt++ ] );
}

