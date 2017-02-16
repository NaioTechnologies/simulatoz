#include "ApiGprsPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiGprsPacket::ApiGprsPacket( )
{

}

//=============================================================================
//
ApiGprsPacket::ApiGprsPacket( GprsCommandeType gprsCommandeType_ ) : gprsCommandeType{ gprsCommandeType_ }
{

}

//=============================================================================
//
ApiGprsPacket::ApiGprsPacket( uint16_t port_, std::string adress_ ) : gprsCommandeType( GprsCommandeType::OPEN_CONNECTION ), port( port_ ), adress( adress_ )
{

}

//=============================================================================
//
ApiGprsPacket::ApiGprsPacket( GprsCommandeType gprsCommandeType_, uint8_t *buffer, uint bufferSize )
		: gprsCommandeType{ gprsCommandeType_ }
{
	dataPtr = cl_copy::unique_buffer( buffer, bufferSize, false );
}

//=============================================================================
//
ApiGprsPacket::~ApiGprsPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiGprsPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer;

	if( gprsCommandeType == GprsCommandeType::LAST_COMMAND_OK or gprsCommandeType == GprsCommandeType::LAST_COMMAND_KO or gprsCommandeType == GprsCommandeType::CLOSE_CONNECTION )
	{
		buffer = cl_copy::unique_buffer( 1 );

		(*buffer)[cpt++] = static_cast<uint8_t>( gprsCommandeType );
	}
	else if( gprsCommandeType == GprsCommandeType::OPEN_CONNECTION )
	{
		buffer = cl_copy::unique_buffer( 1 + 2 + 255 );

		(*buffer)[cpt++] = static_cast<uint8_t>( gprsCommandeType );

		cl::u8Array< 2 > encodedPort = cl::u16_to_u8Array( port );

		(*buffer)[cpt++] = static_cast<uint8_t>( encodedPort[ 0 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedPort[ 1 ] );

		for( uint i = 0; i < 255 ; i++ )
		{
			if( i < adress.length() )
			{
				(*buffer)[cpt++] = static_cast<uint8_t>( adress[i] );
			}
			else
			{
				(*buffer)[cpt++] = static_cast<uint8_t>( '\0' );
			}

		}
	}
	else if( gprsCommandeType == GprsCommandeType::SEND_DATA or gprsCommandeType == GprsCommandeType::DATA_RECEIVED )
	{
		buffer = cl_copy::unique_buffer( 1 + 2 + dataPtr->size() );

		(*buffer)[cpt++] = static_cast<uint8_t>( gprsCommandeType );

		cl::u8Array<2> encodedSize = cl::u16_to_u8Array(static_cast<uint16_t >( dataPtr->size()));

		(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[0] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedSize[1] );

		for ( uint i = 0; i < dataPtr->size(); i++ )
		{
			(*buffer)[cpt++] = static_cast<uint8_t>( dataPtr->at(i));
		}
	}
	else
	{
		buffer = cl_copy::unique_buffer( 1);

		(*buffer)[cpt++] = static_cast<uint8_t>( GprsCommandeType::LAST_COMMAND_KO );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiGprsPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	gprsCommandeType = static_cast<GprsCommandeType>( buffer[ cpt++ ] );

	if( gprsCommandeType == GprsCommandeType::OPEN_CONNECTION )
	{
		cl::u8Array< 2 > encodedPort;

		encodedPort[0] = buffer[ cpt++ ];
		encodedPort[1] = buffer[ cpt++ ];

		port = cl::u8Array_to_u16( encodedPort );

		bool stop = false;
		for( uint i = 0; i < 255 ; i++ )
		{
			char c = static_cast<char>( buffer[ cpt++ ] );

			if( c == '\0' )
				stop = true;

			if( !stop )
				adress += c;
		}
	}
	else if( gprsCommandeType == GprsCommandeType::SEND_DATA or gprsCommandeType == GprsCommandeType::DATA_RECEIVED )
	{
		cl::u8Array< 2 > encodedSize;

		encodedSize[0] = buffer[ cpt++ ];
		encodedSize[1] = buffer[ cpt++ ];

		uint16_t dataSize = cl::u8Array_to_u16( encodedSize );

		dataPtr = cl_copy::unique_buffer( dataSize );

		for( uint i = 0 ; i < dataSize ; i++ )
		{
			(*dataPtr)[i] = buffer[ cpt++ ];
		}
	}
}
