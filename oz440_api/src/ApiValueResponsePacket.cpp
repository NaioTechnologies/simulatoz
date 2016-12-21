#include "ApiValueResponsePacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiValueResponsePacket::ApiValueResponsePacket( )
{

}

//=============================================================================
//
ApiValueResponsePacket::ApiValueResponsePacket( uint8_t id_, KeyPressedType keyPressedType_, int16_t selectedValue_ ) :
	id{ id_ },
	keyPressedType{ keyPressedType_ },
	selectedValue{ selectedValue_ }
{
}

//=============================================================================
//
ApiValueResponsePacket::~ApiValueResponsePacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiValueResponsePacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 1 + 2 );

	(*buffer)[cpt++] = static_cast<uint8_t>( id );
	(*buffer)[cpt++] = static_cast<uint8_t>( keyPressedType );

	cl::u8Array< 2 > encodedValue = cl::i16_to_u8Array( selectedValue );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedValue[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedValue[ 1 ] );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiValueResponsePacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	id = static_cast<uint8_t>( buffer[ cpt++ ] );
	keyPressedType = static_cast<KeyPressedType>( buffer[ cpt++ ] );

	cl::u8Array<2> encodedValue;
	encodedValue[0] = buffer[cpt++];
	encodedValue[1] = buffer[cpt++];
	selectedValue = cl::u8Array_to_i16( encodedValue );
}

