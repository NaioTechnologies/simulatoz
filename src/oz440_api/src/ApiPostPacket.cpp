#include "ApiPostPacket.hpp"
#include "vitals/CLByteConversion.h"

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
cl_copy::BufferUPtr ApiPostPacket::encode()
{
	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + (postList.size() * (1 + 1 + 4 + 4)) );

	uint cpt = 0;

	(*buffer)[cpt++] = static_cast<uint8_t>( postList.size() );

	for( auto&& post: postList )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( post.sourceCamera );

		(*buffer)[cpt++] = static_cast<uint8_t>( post.postType );

		cl::u8Array< 4 > encodedX = cl::float_to_u8Array( post.x ) ;
		cl::u8Array< 4 > encodedY = cl::float_to_u8Array( post.y ) ;

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
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	postList.clear();

	uint8_t postCount = static_cast<uint8_t>( buffer[cpt++] );

	for( int i = 0 ; i < postCount ; i++ )
	{
		SourceCamera sourceCamera = static_cast<SourceCamera>( buffer[cpt++] );
		PostType postType = static_cast<PostType>( buffer[cpt++] );

		cl::u8Array< 4 > encodedX;
		encodedX[0] = buffer[cpt++];
		encodedX[1] = buffer[cpt++];
		encodedX[2] = buffer[cpt++];
		encodedX[3] = buffer[cpt++];

		float x = cl::u8Array_to_float( encodedX );

		cl::u8Array< 4 > encodedY;
		encodedY[0] = buffer[cpt++];
		encodedY[1] = buffer[cpt++];
		encodedY[2] = buffer[cpt++];
		encodedY[3] = buffer[cpt++];

		float y = cl::u8Array_to_float( encodedY );

		postList.push_back( Post( sourceCamera, postType, x, y ) );
	}
}
