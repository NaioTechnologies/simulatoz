#include "../include/oz440_api/ApiAutoStatusPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

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
cl::BufferUPtr ApiAutoStatusPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( autoStatusType );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiAutoStatusPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	autoStatusType = static_cast< AutoStatusType >( buffer[ cpt++ ] );
}
