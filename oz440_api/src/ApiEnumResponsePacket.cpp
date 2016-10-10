#include "../include/oz440_api/ApiEnumResponsePacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"
#include "/usr/include/stdint.h"

//=============================================================================
//
ApiEnumResponsePacket::ApiEnumResponsePacket( )
{

}

//=============================================================================
//
ApiEnumResponsePacket::ApiEnumResponsePacket( uint8_t id_, KeyPressedType keyPressedType_, uint8_t selectedOption_ ) :
	id{ id_ },
	keyPressedType{ keyPressedType_ },
	selectedOption{ selectedOption_ }
{
}

//=============================================================================
//
ApiEnumResponsePacket::~ApiEnumResponsePacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiEnumResponsePacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 + 1 + 1 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( id );
	(*buffer)[cpt++] = static_cast<uint8_t>( keyPressedType );
	(*buffer)[cpt++] = static_cast<uint8_t>( selectedOption );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiEnumResponsePacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	id = static_cast<uint8_t>( buffer[ cpt++ ] );
	keyPressedType = static_cast<KeyPressedType>( buffer[ cpt++ ] );
	selectedOption = static_cast<char>( buffer[ cpt++ ] );
}

