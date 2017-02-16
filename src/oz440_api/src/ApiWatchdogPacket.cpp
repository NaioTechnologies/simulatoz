#include "ApiWatchdogPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiWatchdogPacket::ApiWatchdogPacket( )
{

}

//=============================================================================
//
ApiWatchdogPacket::ApiWatchdogPacket( uint8_t fooValue_ )
	: 	fooValue{ fooValue_ }
{

}

//=============================================================================
//
ApiWatchdogPacket::~ApiWatchdogPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiWatchdogPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( fooValue );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiWatchdogPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	fooValue = static_cast< uint8_t >( buffer[ cpt++ ] );
}
