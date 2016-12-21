#include "ApiAutoStatusPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiAutoStatusPacket::ApiAutoStatusPacket( )
{

}

//=============================================================================
//
ApiAutoStatusPacket::ApiAutoStatusPacket( AutoStatusType autoStatusType_ )
	: 	autoStatusType{ autoStatusType_ }
{

}

//=============================================================================
//
ApiAutoStatusPacket::~ApiAutoStatusPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiAutoStatusPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( autoStatusType );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiAutoStatusPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	autoStatusType = static_cast< AutoStatusType >( buffer[ cpt++ ] );
}
