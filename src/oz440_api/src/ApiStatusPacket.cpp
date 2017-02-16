#include "ApiStatusPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiStatusPacket::ApiStatusPacket( )
{

}

//=============================================================================
//
ApiStatusPacket::ApiStatusPacket( bool imuReseted_, double theta_, int32_t odoFR_, int32_t odoRR_, int32_t odoRL_, int32_t odoFL_, double positionX_, double positionY_, double distance_, uint8_t actuatorPosition_, uint8_t battery_, int16_t magX_, int16_t magY_, int16_t magZ_ )
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
		battery{ battery_ },
		magX{ magX_ },
		magY{ magY_ },
		magZ{ magZ_ }
{

}

//=============================================================================
//
ApiStatusPacket::~ApiStatusPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiStatusPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 8 + 16 + 16 + 8 + 1 + 1 + 2 + 2 + 2 );

	(*buffer)[cpt++] = static_cast<uint8_t>( imuReseted );

	cl::u8Array< 8 > encodedHeading = cl::double_to_u8Array( theta );

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

	cl::u8Array< 8 > encodedPositionX = cl::double_to_u8Array( positionX );
	cl::u8Array< 8 > encodedPositionY = cl::double_to_u8Array( positionY );

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedPositionX[ i ] );
	}

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedPositionY[ i ] );
	}

	cl::u8Array< 8 > encodedDistance = cl::double_to_u8Array( distance );

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedDistance[ i ] );
	}

	(*buffer)[cpt++] = actuatorPosition;

	(*buffer)[cpt++] = battery;

	cl::u8Array< 2 > encodedMagX = cl::i16_to_u8Array( magX );
	cl::u8Array< 2 > encodedMagY = cl::i16_to_u8Array( magY );
	cl::u8Array< 2 > encodedMagZ = cl::i16_to_u8Array( magZ );

	for( uint i = 0; i < 2 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedMagX[ i ] );
	}

	for( uint i = 0; i < 2 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedMagY[ i ] );
	}

	for( uint i = 0; i < 2 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedMagZ[ i ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiStatusPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	imuReseted = static_cast<bool>( buffer[ cpt++ ] );

	cl::u8Array< 8 > encodedHeading;

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedHeading[ i ] =  buffer[ cpt++ ];
	}

	theta = cl::u8Array_to_double( encodedHeading );

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

	positionX = cl::u8Array_to_double( encodedPositionX );
	positionY = cl::u8Array_to_double( encodedPositionY );

	cl::u8Array< 8 > encodedDistance;

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedDistance[ i ] =  buffer[ cpt++ ];
	}

	distance = cl::u8Array_to_double( encodedDistance );


	actuatorPosition = buffer[ cpt++ ];

	battery = buffer[ cpt++ ];

	cl::u8Array< 2 > encodedMagX;
	cl::u8Array< 2 > encodedMagY;
	cl::u8Array< 2 > encodedMagZ;

	for( uint i = 0; i < 2 ; i++ )
	{
		encodedMagX[ i ] =  buffer[ cpt++ ];
	}

	for( uint i = 0; i < 2 ; i++ )
	{
		encodedMagY[ i ] =  buffer[ cpt++ ];
	}

	for( uint i = 0; i < 2 ; i++ )
	{
		encodedMagZ[ i ] =  buffer[ cpt++ ];
	}

	magX = cl::u8Array_to_i16( encodedMagX );
	magY = cl::u8Array_to_i16( encodedMagY );
	magZ = cl::u8Array_to_i16( encodedMagZ );
}
