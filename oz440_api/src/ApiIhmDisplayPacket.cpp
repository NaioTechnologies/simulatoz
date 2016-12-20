#include "../include/oz440_api/ApiIhmDisplayPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
ApiIhmDisplayPacket::ApiIhmDisplayPacket( )
{

}

//=============================================================================
//
ApiIhmDisplayPacket::ApiIhmDisplayPacket( const char topLine_[20], const char bottomLine_[20] )
{
	for( uint i = 0 ; i < 20 ; i++ )
	{
		topLine[ i ] = topLine_[ i ];
	}

	for( uint i = 0 ; i < 20 ; i++ )
	{
		bottomLine[ i ] = bottomLine_[ i ];
	}
}

//=============================================================================
//
ApiIhmDisplayPacket::~ApiIhmDisplayPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiIhmDisplayPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 20 + 20 ) );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( topLine[ i ] );
	}

	for( uint i = 0 ; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( bottomLine[ i ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiIhmDisplayPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	for( uint i = 0 ; i < 20 ; i++ )
	{
		topLine[ i ] = static_cast<char>( buffer[ cpt++ ] );
	}

	for( uint i = 0 ; i < 20 ; i++ )
	{
		bottomLine[ i ] = static_cast<char>( buffer[ cpt++ ] );
	}

}

