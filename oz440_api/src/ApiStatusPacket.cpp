#include "../include/oz440_api/ApiStatusPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
ApiStatusPacket::ApiStatusPacket( )
{

}

//=============================================================================
//
ApiStatusPacket::ApiStatusPacket( bool imuReseted_, double theta_, int32_t odoFR_, int32_t odoRR_, int32_t odoRL_, int32_t odoFL_, double positionX_, double positionY_, double distance_, uint8_t actuatorPosition_, uint8_t battery_ )
	: 	imuReseted{ imuReseted_ },
		theta{ theta_ },
		odoFR{ odoFR_ },
		odoRR{ odoRR_ },
		odoRL{ odoRL_ },
		odoFL{ odoFL_ },
		positionX{ positionX_ },
		positionY{ positionY_ },
		distance{ distance_ },
		actuatorPosition{ actuatorPosition_ },
		battery{ battery_ }
{

}

//=============================================================================
//
ApiStatusPacket::~ApiStatusPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiStatusPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 + 8 + 16 + 16 + 8 + 1 + 1 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( imuReseted );

	cl::u8Array< 8 > encodedHeading = cl::float64_to_u8Array( theta );

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedHeading[ i ] );
	}

	cl::u8Array< 4 > encodedFR = cl::i32_to_u8Array( odoFR );
	cl::u8Array< 4 > encodedRR = cl::i32_to_u8Array( odoRR );
	cl::u8Array< 4 > encodedRL = cl::i32_to_u8Array( odoRL );
	cl::u8Array< 4 > encodedFL = cl::i32_to_u8Array( odoFL );

	for( uint i = 0; i < 4 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedFR[ i ] );
	}

	for( uint i = 0; i < 4 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRR[ i ] );
	}

	for( uint i = 0; i < 4 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRL[ i ] );
	}

	for( uint i = 0; i < 4 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedFL[ i ] );
	}

	cl::u8Array< 8 > encodedPositionX = cl::float64_to_u8Array( positionX );
	cl::u8Array< 8 > encodedPositionY = cl::float64_to_u8Array( positionY );

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedPositionX[ i ] );
	}

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedPositionY[ i ] );
	}

	cl::u8Array< 8 > encodedDistance = cl::float64_to_u8Array( distance );

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedDistance[ i ] );
	}

	(*buffer)[cpt++] = actuatorPosition;

	(*buffer)[cpt++] = battery;


	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiStatusPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	imuReseted = static_cast<bool>( buffer[ cpt++ ] );

	cl::u8Array< 8 > encodedHeading;

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedHeading[ i ] =  buffer[ cpt++ ];
	}

	theta = cl::u8Array_to_float64( encodedHeading );

	cl::u8Array< 4 > encodedFR;
	cl::u8Array< 4 > encodedRR;
	cl::u8Array< 4 > encodedRL;
	cl::u8Array< 4 > encodedFL;

	for( uint i = 0; i < 4 ; i++ )
	{
		encodedFR[ i ] =  buffer[ cpt++ ];
	}

	for( uint i = 0; i < 4 ; i++ )
	{
		encodedRR[ i ] =  buffer[ cpt++ ];
	}

	for( uint i = 0; i < 4 ; i++ )
	{
		encodedRL[ i ] =  buffer[ cpt++ ];
	}

	for( uint i = 0; i < 4 ; i++ )
	{
		encodedFL[ i ] =  buffer[ cpt++ ];
	}

	odoFR = cl::u8Array_to_i32( encodedFR );
	odoRR = cl::u8Array_to_i32( encodedRR );
	odoRL = cl::u8Array_to_i32( encodedRL );
	odoFL = cl::u8Array_to_i32( encodedFL );

	cl::u8Array< 8 > encodedPositionX;
	cl::u8Array< 8 > encodedPositionY;

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedPositionX[ i ] =  buffer[ cpt++ ];
	}

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedPositionY[ i ] =  buffer[ cpt++ ];
	}

	positionX = cl::u8Array_to_float64( encodedPositionX );
	positionY = cl::u8Array_to_float64( encodedPositionY );

	cl::u8Array< 8 > encodedDistance;

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedDistance[ i ] =  buffer[ cpt++ ];
	}

	distance = cl::u8Array_to_float64( encodedDistance );


	actuatorPosition = buffer[ cpt++ ];

	battery = buffer[ cpt++ ];
}
