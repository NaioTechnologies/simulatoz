#include "ApiEnumResponsePacket.hpp"
#include "vitals/CLByteConversion.h"

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
cl_copy::BufferUPtr ApiEnumResponsePacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 1 + 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( id );
	(*buffer)[cpt++] = static_cast<uint8_t>( keyPressedType );
	(*buffer)[cpt++] = static_cast<uint8_t>( selectedOption );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiEnumResponsePacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	id = static_cast<uint8_t>( buffer[ cpt++ ] );
	keyPressedType = static_cast<KeyPressedType>( buffer[ cpt++ ] );
	selectedOption =  buffer[ cpt++ ] ;
}

