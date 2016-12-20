#include "../include/oz440_api/HaStereoCameraPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
HaStereoCameraPacket::HaStereoCameraPacket( )
: dataBuffer{ cl::shared_buffer( 0 ) }
{

}

//=============================================================================
//
HaStereoCameraPacket::HaStereoCameraPacket( uint8_t  dataBuffer_[541440] )
    : dataBuffer{ cl::shared_buffer( 541440 ) }
{
    for(int i=0; i<541400; i++) {
        dataBuffer->at(i) = dataBuffer_[i];
    }
}

//=============================================================================
//
HaStereoCameraPacket::~HaStereoCameraPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr HaStereoCameraPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 4 + 541440 ) );

	cl::u8Array<4> encodedSize = cl::u32_to_u8Array(static_cast<uint32_t >( 541440 ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[0] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[1] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[2] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[3] );
	for ( uint i = 0; i < 541440; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( dataBuffer->at(i));
	}
	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void HaStereoCameraPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	cl::u8Array< 4 > encodedSize;

	encodedSize[0] = buffer[ cpt++ ];
	encodedSize[1] = buffer[ cpt++ ];
	encodedSize[2] = buffer[ cpt++ ];
	encodedSize[3] = buffer[ cpt++ ];

	uint32_t dataSize = cl::u8Array_to_u32( encodedSize );

	dataBuffer = cl::unique_buffer( static_cast<size_t>( dataSize ) );

	for( uint i = 0 ; i < dataSize ; i++ )
	{
		(*dataBuffer)[i] = buffer[ cpt++ ];
	}
}
