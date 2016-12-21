#include "HaCanPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
HaCanPacket::HaCanPacket( )
{

}

//=============================================================================
//
HaCanPacket::HaCanPacket(  uint8_t dataBuffer_[21], uint8_t dataBufferSize_ )
{
	dataBufferSize = dataBufferSize_;

	if( dataBufferSize > 21 )
	{
		dataBufferSize = 21;
	}

	for( uint i = 0 ; i < dataBufferSize ; i++ )
	{
		dataBuffer[ i ] = dataBuffer_[ i ];
	}
}

//=============================================================================
//
HaCanPacket::~HaCanPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr HaCanPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + dataBufferSize );

	(*buffer)[cpt++] = static_cast<uint8_t>( dataBufferSize );

	for( uint i = 0 ; i < dataBufferSize ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( dataBuffer[ i ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaCanPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	dataBufferSize =  static_cast< uint8_t >( buffer[ cpt++ ] );

	for( uint i = 0 ; i < dataBufferSize ; i++ )
	{
		dataBuffer[ i ] = static_cast< uint8_t >( buffer[ cpt++ ] );
	}
}

