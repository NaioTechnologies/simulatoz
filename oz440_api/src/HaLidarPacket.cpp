#include "HaLidarPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
HaLidarPacket::HaLidarPacket( )
{

}

//=============================================================================
//
HaLidarPacket::HaLidarPacket( uint16_t distance_[271], uint8_t albedo_[271] )
{
	for( uint i = 0 ; i < 271 ; i++ )
	{
		distance[ i ] = distance_[ i ];
		albedo[ i ] = albedo_[ i ];
	}
}

//=============================================================================
//
HaLidarPacket::~HaLidarPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr HaLidarPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( ( 271 * 2 ) + 271 );

	for( uint i = 0 ; i < 271 ; i++ )
	{
		cl::u8Array< 2 > encodedDistance = cl::u16_to_u8Array( distance[i] );

		(*buffer)[cpt++] = static_cast<uint8_t>( encodedDistance[ 0 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedDistance[ 1 ] );
	}

	for( uint i = 0 ; i < 271 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( albedo[ i ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaLidarPacket::decode( uint8_t *buffer, uint bufferSize )
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

	for( uint i = 0 ; i < 271 ; i++ )
	{
		albedo[ i ] = buffer[cpt++];
	}

}
