#include "../include/oz440_api/HaLedPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
HaLedPacket::HaLedPacket( )
{

}

//=============================================================================
//
HaLedPacket::HaLedPacket( uint8_t led_ )
	: 	led{ led_ }
{

}

//=============================================================================
//
HaLedPacket::~HaLedPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr HaLedPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( led );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaLedPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	led = static_cast< uint8_t >( buffer[ cpt++ ] );
}

