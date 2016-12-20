#include "../include/oz440_api/ApiCommandPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
ApiCommandPacket::ApiCommandPacket( )
{

}

//=============================================================================
//
ApiCommandPacket::ApiCommandPacket( CommandType commandType_ )
	: 	commandType{ commandType_ }
{

}

//=============================================================================
//
ApiCommandPacket::~ApiCommandPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiCommandPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( commandType );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiCommandPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	commandType = static_cast< CommandType >( buffer[ cpt++ ] );
}
