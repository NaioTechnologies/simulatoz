#include "ApiGpsPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiGpsPacket::ApiGpsPacket( )
{

}

//=============================================================================
//
ApiGpsPacket::ApiGpsPacket( GpsType gpsType_, ulong time_, double lat_, double lon_, double alt_,
							uint8_t unit_, uint8_t satUsed_, uint8_t quality_, double groundSpeed_,
							double trackOrientation_ )
	: gpsType{ gpsType_ },
		time{ time_ },
		lat{ lat_ },
		lon{ lon_ },
		alt{ alt_ },
		unit{ unit_ },
		satUsed{ satUsed_ },
	    quality{ quality_ },
	    groundSpeed{ groundSpeed_ },
	    trackOrientation{ trackOrientation_ }
{

}

//=============================================================================
//
ApiGpsPacket::~ApiGpsPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiGpsPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 8 + 8 + 8 + 8 + 1 + 1 + 1 + 8 + 8 );

	cl::u8Array< 8 > encodedTime = cl::u64_to_u8Array( time );
	cl::u8Array< 8 > encodedLat = cl::double_to_u8Array( lat );
	cl::u8Array< 8 > encodedLon = cl::double_to_u8Array( lon );
	cl::u8Array< 8 > encodedAlt = cl::double_to_u8Array( alt );
	cl::u8Array< 8 > encodedGroundSpeed = cl::double_to_u8Array( groundSpeed );
	cl::u8Array< 8 > encodedTrackOrientation = cl::double_to_u8Array( trackOrientation );

	(*buffer)[cpt++] = static_cast<uint8_t>( gpsType );

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedTime[ i ] );
	}

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedLat[ i ] );
	}

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedLon[ i ] );
	}

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedAlt[ i ] );
	}

	(*buffer)[cpt++] = static_cast<uint8_t>( unit );
	(*buffer)[cpt++] = static_cast<uint8_t>( satUsed );
	(*buffer)[cpt++] = static_cast<uint8_t>( quality );

	for( uint i = 0; i < 8 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedGroundSpeed[ i ] );
	}

	for( uint i = 0; i < 8; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedTrackOrientation[i] );
	}


	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiGpsPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	cl::u8Array< 8 > encodedTime;
	cl::u8Array< 8 > encodedLat;
	cl::u8Array< 8 > encodedLon;
	cl::u8Array< 8 > encodedAlt;
	cl::u8Array< 8 > encodedGroundSpeed;
	cl::u8Array< 8 > encodedTrackOrientation;

	// #######################

	gpsType = static_cast<GpsType>( buffer[ cpt++ ] );

	// #######################

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedTime[ i ] =  buffer[ cpt++ ];
	}

	time = cl::u8Array_to_u64( encodedTime );

	// #######################

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedLat[ i ] =  buffer[ cpt++ ];
	}

	lat = cl::u8Array_to_double( encodedLat );

	// #######################

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedLon[ i ] =  buffer[ cpt++ ];
	}

	lon = cl::u8Array_to_double( encodedLon );

	// #######################

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedAlt[ i ] =  buffer[ cpt++ ];
	}

	alt = cl::u8Array_to_double( encodedAlt );

	// #######################

	unit =  buffer[ cpt++ ];
	satUsed =  buffer[ cpt++ ];
	quality =  buffer[ cpt++ ];

	// #######################

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedGroundSpeed[ i ] =  buffer[ cpt++ ];
	}

	groundSpeed = cl::u8Array_to_double( encodedGroundSpeed );

	// #######################

	for( uint i = 0; i < 8 ; i++ )
	{
		encodedTrackOrientation[ i ] =  buffer[ cpt++ ];
	}

	trackOrientation = cl::u8Array_to_double( encodedTrackOrientation );
}
