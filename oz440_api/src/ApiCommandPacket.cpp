#include "ApiCommandPacket.hpp"
#include "vitals/CLByteConversion.h"

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
cl_copy::BufferUPtr ApiCommandPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( commandType );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiCommandPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	commandType = static_cast< CommandType >( buffer[ cpt++ ] );
}
