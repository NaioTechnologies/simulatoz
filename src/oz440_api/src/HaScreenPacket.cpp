#include "HaScreenPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
HaScreenPacket::HaScreenPacket( )
{

}

//=============================================================================
//
HaScreenPacket::HaScreenPacket( char topLine_[16], char bottomLine_[16] )
{
	for( uint i = 0 ; i < 16 ; i++ )
	{
		topLine[ i ] = topLine_[ i ];
	}

	for( uint i = 0 ; i < 16 ; i++ )
	{
		bottomLine[ i ] = bottomLine_[ i ];
	}
}

//=============================================================================
//
HaScreenPacket::~HaScreenPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr HaScreenPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 16 + 16 );

	for( uint i = 0 ; i < 16 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( topLine[ i ] );
	}

	for( uint i = 0 ; i < 16 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( bottomLine[ i ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaScreenPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	for( uint i = 0 ; i < 16 ; i++ )
	{
		topLine[ i ] = static_cast<char>( buffer[ cpt++ ] );
	}

	for( uint i = 0 ; i < 16 ; i++ )
	{
		bottomLine[ i ] = static_cast<char>( buffer[ cpt++ ] );
	}

}

