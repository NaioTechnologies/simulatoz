#include "HaKeypadPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
HaKeypadPacket::HaKeypadPacket( )
{

}

//=============================================================================
//
HaKeypadPacket::HaKeypadPacket( uint8_t keypad_ )
	: 	keypad{ keypad_ }
{

}

//=============================================================================
//
HaKeypadPacket::~HaKeypadPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr HaKeypadPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( keypad );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaKeypadPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	keypad = static_cast< uint8_t >( buffer[ cpt++ ] );
}

