#include "../include/oz440_api/ApiMotorsPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
ApiMotorsPacket::ApiMotorsPacket( )
{

}

//=============================================================================
//
ApiMotorsPacket::ApiMotorsPacket( int8_t left_, int8_t right_ )
	: 	left{ left_ },
		right{ right_ }
{

}

//=============================================================================
//
ApiMotorsPacket::~ApiMotorsPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiMotorsPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 2 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( left );
	(*buffer)[cpt++] = static_cast<uint8_t>( right );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiMotorsPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	left = static_cast< int8_t >( buffer[ cpt++ ] );
	right = static_cast< int8_t >( buffer[ cpt++ ] );
}

