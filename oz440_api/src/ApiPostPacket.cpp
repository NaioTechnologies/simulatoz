#include "../include/oz440_api/ApiPostPacket.hpp"
#include "../include/oz440_api/CLByteConversion.h"

//=============================================================================
//
ApiPostPacket::ApiPostPacket( )
: postList{ }
{

}

//=============================================================================
//
ApiPostPacket::~ApiPostPacket( )
{

}

//=============================================================================
//
cl::BufferUPtr ApiPostPacket::encode()
{
	cl::BufferUPtr buffer = cl::unique_buffer( static_cast<size_t>( 1 + ( postList.size() * ( 1 + 4 + 4 ) ) ) );

	uint cpt = 0;

	(*buffer)[cpt++] = static_cast<uint8_t>( postList.size() );

	for( auto&& post: postList )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( post.postType );

		cl::u8Array< 4 > encodedX = cl::float32_to_u8Array( post.x ) ;
		cl::u8Array< 4 > encodedY = cl::float32_to_u8Array( post.y ) ;

		(*buffer)[cpt++] = encodedX[0];
		(*buffer)[cpt++] = encodedX[1];
		(*buffer)[cpt++] = encodedX[2];
		(*buffer)[cpt++] = encodedX[3];

		(*buffer)[cpt++] = encodedY[0];
		(*buffer)[cpt++] = encodedY[1];
		(*buffer)[cpt++] = encodedY[2];
		(*buffer)[cpt++] = encodedY[3];
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiPostPacket::decode( uint8_t *buffer, uint bufferSize )
{
	ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	postList.clear();

	uint8_t postCount = static_cast<uint8_t>( buffer[cpt++] );

	for( int i = 0 ; i < postCount ; i++ )
	{
		PostType postType = static_cast<PostType>( buffer[cpt++] );

		cl::u8Array< 4 > encodedX;
		encodedX[0] = buffer[cpt++];
		encodedX[1] = buffer[cpt++];
		encodedX[2] = buffer[cpt++];
		encodedX[3] = buffer[cpt++];

		float x = cl::u8Array_to_float32( encodedX );

		cl::u8Array< 4 > encodedY;
		encodedY[0] = buffer[cpt++];
		encodedY[1] = buffer[cpt++];
		encodedY[2] = buffer[cpt++];
		encodedY[3] = buffer[cpt++];

		float y = cl::u8Array_to_float32( encodedY );

		postList.push_back( Post( postType, x, y ) );
	}
}
