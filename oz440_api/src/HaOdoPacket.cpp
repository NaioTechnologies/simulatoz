#include "HaOdoPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
HaOdoPacket::HaOdoPacket( )
{

}

//=============================================================================
//
HaOdoPacket::HaOdoPacket( uint8_t fr_, uint8_t rr_, uint8_t rl_, uint8_t fl_ )
	: 	fr{ fr_ },
		rr{ rr_ },
		rl{ rl_ },
		fl{ fl_ }
{

}

//=============================================================================
//
HaOdoPacket::~HaOdoPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr HaOdoPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 1 + 1 + 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( fr );
	(*buffer)[cpt++] = static_cast<uint8_t>( rr );
	(*buffer)[cpt++] = static_cast<uint8_t>( rl );
	(*buffer)[cpt++] = static_cast<uint8_t>( fl );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaOdoPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	fr = static_cast< uint8_t >( buffer[ cpt++ ] );
	rr = static_cast< uint8_t >( buffer[ cpt++ ] );
	rl = static_cast< uint8_t >( buffer[ cpt++ ] );
	fl = static_cast< uint8_t >( buffer[ cpt++ ] );
}

