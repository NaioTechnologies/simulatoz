#include "../include/oz440_api/ApiStereoCameraPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
ApiStereoCameraPacket::ApiStereoCameraPacket( )
{

}

//=============================================================================
//
ApiStereoCameraPacket::ApiStereoCameraPacket(  ImageType imageType_, cl::BufferUPtr dataBuffer_ )
{
	imageType = imageType_;
	dataBuffer = std::move( dataBuffer_ );
}

//=============================================================================
//
ApiStereoCameraPacket::~ApiStereoCameraPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiStereoCameraPacket::encode()
{
	uint cpt = 0;

	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 + 4 + dataBuffer->size() ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( imageType );

	cl::u8Array<4> encodedSize = cl::u32_to_u8Array(static_cast<uint32_t >( dataBuffer->size() ) );

	(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[0] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[1] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[2] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[3] );

	for ( uint i = 0; i < dataBuffer->size(); i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( dataBuffer->at(i) );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiStereoCameraPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	imageType = static_cast<ImageType >( buffer[ cpt++ ] );

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
