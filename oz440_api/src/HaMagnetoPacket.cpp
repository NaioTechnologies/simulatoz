#include "HaMagnetoPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
HaMagnetoPacket::HaMagnetoPacket( )
{

}

//=============================================================================
//
HaMagnetoPacket::HaMagnetoPacket( int16_t x_, int16_t y_, int16_t z_  )
	: 	x{ x_ },
		y{ y_ },
		z{ z_ }
{

}

//=============================================================================
//
HaMagnetoPacket::~HaMagnetoPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr HaMagnetoPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 2 + 2 + 2 );

	cl::u8Array< 2 > encodedX = cl::i16_to_u8Array( x );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedX[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedX[ 1 ] );

	cl::u8Array< 2 > encodedY = cl::i16_to_u8Array( y );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedY[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedY[ 1 ] );

	cl::u8Array< 2 > encodedZ = cl::i16_to_u8Array( z );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedZ[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedZ[ 1 ] );


	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaMagnetoPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	cl::u8Array<2> encodedX;
	encodedX[0] = buffer[cpt++];
	encodedX[1] = buffer[cpt++];
	x = cl::u8Array_to_i16( encodedX );

	cl::u8Array<2> encodedY;
	encodedY[0] = buffer[cpt++];
	encodedY[1] = buffer[cpt++];
	y = cl::u8Array_to_i16( encodedY );

	cl::u8Array<2> encodedZ;
	encodedZ[0] = buffer[cpt++];
	encodedZ[1] = buffer[cpt++];
	z = cl::u8Array_to_i16( encodedZ );
}

