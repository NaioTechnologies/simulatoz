#include "../include/oz440_api/HaSpeakerPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

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
cl::BufferUPtr HaSpeakerPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 + 1 + 1 + 1 + 1 + 1 + 1 ) );

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
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	uint8_t reserved1 = static_cast< uint8_t >( buffer[ cpt++ ] );
	uint8_t reserved2 = static_cast< uint8_t >( buffer[ cpt++ ] );

	duration = static_cast< uint8_t >( buffer[ cpt++ ] );

	uint8_t reserved3 = static_cast< uint8_t >( buffer[ cpt++ ] );
	uint8_t reserved4 = static_cast< uint8_t >( buffer[ cpt++ ] );

	volume = static_cast< uint8_t >( buffer[ cpt++ ] );

	uint8_t reserved5 = static_cast< uint8_t >( buffer[ cpt++ ] );

	ignore( reserved1 );
	ignore( reserved2 );
	ignore( reserved3 );
	ignore( reserved4 );
	ignore( reserved5 );
}

