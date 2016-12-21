#include "HaDS4RemotePacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
HaDS4RemotePacket::HaDS4RemotePacket( )
{

}

//=============================================================================
//
HaDS4RemotePacket::HaDS4RemotePacket(  uint8_t battery_, int8_t leftAnalogX_, int8_t leftAnalogY_, int8_t rightAnalogX_, int8_t rightAnalogY_,
									   uint8_t buttons_, uint8_t l1l3r1r3_, uint8_t l2_, uint8_t r2_, int8_t accelX_, int8_t accelY_, int8_t accelZ_,
									   int8_t gyroX_, int8_t gyroY_, int8_t gyroZ_ )
	: 	 battery{ battery_ },
		 leftAnalogX{ leftAnalogX_ },
		 leftAnalogY{ leftAnalogY_ },
		 rightAnalogX{ rightAnalogX_ },
		 rightAnalogY{ rightAnalogY_ },
		 buttons{ buttons_ },
		 l1l3r1r3{ l1l3r1r3_ },
		 l2{ l2_ },
		 r2{ r2_ },
		 accelX{ accelX_ },
		 accelY{ accelY_ },
		 accelZ{ accelZ_ },
		 gyroX{ gyroX_ },
		 gyroY{ gyroY_ },
		 gyroZ{ gyroZ_ }
{

}

//=============================================================================
//
HaDS4RemotePacket::~HaDS4RemotePacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr HaDS4RemotePacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 16 );

	(*buffer)[cpt++] = static_cast<uint8_t>( 2 );

	(*buffer)[cpt++] = static_cast<uint8_t>( battery );
	(*buffer)[cpt++] = static_cast<uint8_t>( leftAnalogX );
	(*buffer)[cpt++] = static_cast<uint8_t>( leftAnalogY );
	(*buffer)[cpt++] = static_cast<uint8_t>( rightAnalogX );
	(*buffer)[cpt++] = static_cast<uint8_t>( rightAnalogY );
	(*buffer)[cpt++] = static_cast<uint8_t>( buttons );
	(*buffer)[cpt++] = static_cast<uint8_t>( l1l3r1r3 );
	(*buffer)[cpt++] = static_cast<uint8_t>( l2 );
	(*buffer)[cpt++] = static_cast<uint8_t>( r2 );
	(*buffer)[cpt++] = static_cast<uint8_t>( accelX );
	(*buffer)[cpt++] = static_cast<uint8_t>( accelY );
	(*buffer)[cpt++] = static_cast<uint8_t>( accelZ );
	(*buffer)[cpt++] = static_cast<uint8_t>( gyroX );
	(*buffer)[cpt++] = static_cast<uint8_t>( gyroY );
	(*buffer)[cpt++] = static_cast<uint8_t>( gyroZ );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaDS4RemotePacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	//uint8_t reserved = static_cast< uint8_t >( buffer[ cpt++ ] );
	cpt++;

	battery = static_cast< uint8_t >( buffer[ cpt++ ] );

	leftAnalogX = static_cast< int8_t >( buffer[ cpt++ ] );
	leftAnalogY = static_cast< int8_t >( buffer[ cpt++ ] );
	rightAnalogX = static_cast< int8_t >( buffer[ cpt++ ] );
	rightAnalogY = static_cast< int8_t >( buffer[ cpt++ ] );

	buttons = static_cast< uint8_t >( buffer[ cpt++ ] );
	l1l3r1r3 = static_cast< uint8_t >( buffer[ cpt++ ] );

	l2 = static_cast< uint8_t >( buffer[ cpt++ ] );
	r2 = static_cast< uint8_t >( buffer[ cpt++ ] );

	accelX = static_cast< int8_t >( buffer[ cpt++ ] );
	accelY = static_cast< int8_t >( buffer[ cpt++ ] );
	accelZ = static_cast< int8_t >( buffer[ cpt++ ] );

	gyroX = static_cast< int8_t >( buffer[ cpt++ ] );
	gyroY = static_cast< int8_t >( buffer[ cpt++ ] );
	gyroZ = static_cast< int8_t >( buffer[ cpt++ ] );
}

