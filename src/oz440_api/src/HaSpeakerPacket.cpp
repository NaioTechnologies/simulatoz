#include "HaSpeakerPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
HaSpeakerPacket::HaSpeakerPacket( )
{

}

//=============================================================================
//
HaSpeakerPacket::HaSpeakerPacket( uint8_t duration_, uint8_t volume_ )
	: 	duration{ duration_ },
        volume{ volume_ }
{

}

//=============================================================================
//
HaSpeakerPacket::~HaSpeakerPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr HaSpeakerPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 1 + 1 + 1 + 1 + 1 + 1 );

	(*buffer)[cpt++] = static_cast<uint8_t>( 1 );
	(*buffer)[cpt++] = static_cast<uint8_t>( 0 );

	(*buffer)[cpt++] = static_cast<uint8_t>( duration );

	(*buffer)[cpt++] = static_cast<uint8_t>( 0 );
	(*buffer)[cpt++] = static_cast<uint8_t>( 0 );

	(*buffer)[cpt++] = static_cast<uint8_t>( volume );

	(*buffer)[cpt++] = static_cast<uint8_t>( 0 );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaSpeakerPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	uint8_t reserved1 = static_cast< uint8_t >( buffer[ cpt++ ] );
	uint8_t reserved2 = static_cast< uint8_t >( buffer[ cpt++ ] );

	duration = static_cast< uint8_t >( buffer[ cpt++ ] );

	uint8_t reserved3 = static_cast< uint8_t >( buffer[ cpt++ ] );
	uint8_t reserved4 = static_cast< uint8_t >( buffer[ cpt++ ] );

	volume = static_cast< uint8_t >( buffer[ cpt++ ] );

	uint8_t reserved5 = static_cast< uint8_t >( buffer[ cpt++ ] );

	util_copy::ignore( reserved1 );
	util_copy::ignore( reserved2 );
	util_copy::ignore( reserved3 );
	util_copy::ignore( reserved4 );
	util_copy::ignore( reserved5 );
}

