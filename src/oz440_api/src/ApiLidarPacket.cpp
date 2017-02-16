#include "ApiLidarPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiLidarPacket::ApiLidarPacket( )
{

}

//=============================================================================
//
ApiLidarPacket::ApiLidarPacket( uint16_t distance_[271] )
{
	for( uint i = 0 ; i < 271 ; i++ )
	{
		distance[ i ] = distance_[ i ];
	}
}

//=============================================================================
//
ApiLidarPacket::~ApiLidarPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiLidarPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 271 * 2 );

	for( uint i = 0 ; i < 271 ; i++ )
	{
		cl::u8Array< 2 > encodedDistance = cl::u16_to_u8Array( distance[i] );

		(*buffer)[cpt++] = static_cast<uint8_t>( encodedDistance[ 0 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedDistance[ 1 ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiLidarPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	for( uint i = 0 ; i < 271 ; i++ )
	{
		cl::u8Array<2> encodedDistance;

		encodedDistance[0] = buffer[cpt++];
		encodedDistance[1] = buffer[cpt++];

		distance[i] = cl::u8Array_to_u16( encodedDistance );
	}
}
