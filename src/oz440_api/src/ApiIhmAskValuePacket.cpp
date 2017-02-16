#include "ApiIhmAskValuePacket.hpp"
#include "vitals/CLByteConversion.h"
#include <string.h>

//=============================================================================
//
ApiIhmAskValuePacket::ApiIhmAskValuePacket( )
{

}

//=============================================================================
//
ApiIhmAskValuePacket::ApiIhmAskValuePacket( uint8_t id_, char topLine_[20], char question_[20], int16_t min_, int16_t max_, int16_t step_, int16_t defaultValue_, char unit_[20] )
{
	id = id_;

	memcpy( topLine, topLine_, 20 );

	memcpy( question, question_, 20 );

	min = min_;
	max = max_;
	step = step_;
	defaultValue = defaultValue_;

	memcpy( unit, unit_, 20 );
}

//=============================================================================
//
ApiIhmAskValuePacket::~ApiIhmAskValuePacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiIhmAskValuePacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 20 + 20 + 2 + 2 + 2 + 2 + 20 );

	(*buffer)[cpt++] = static_cast<uint8_t>( id );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( topLine[ i ] );
	}

	for( uint i = 0 ; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( question[ i ] );
	}

	cl::u8Array< 2 > encodedMin = cl::i16_to_u8Array( min );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedMin[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedMin[ 1 ] );

	cl::u8Array< 2 > encodedMax = cl::i16_to_u8Array( max );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedMax[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedMax[ 1 ] );

	cl::u8Array< 2 > encodedStep = cl::i16_to_u8Array( step );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedStep[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedStep[ 1 ] );

	cl::u8Array< 2 > encodedDefault = cl::i16_to_u8Array( defaultValue );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedDefault[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedDefault[ 1 ] );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( unit[ i ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiIhmAskValuePacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	id =  static_cast<uint8_t>( buffer[ cpt++ ] );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		topLine[ i ] = static_cast<char>( buffer[ cpt++ ] );
	}

	for( uint i = 0 ; i < 20 ; i++ )
	{
		question[ i ] = static_cast<char>( buffer[ cpt++ ] );
	}

	cl::u8Array<2> encodedMin;
	encodedMin[0] = buffer[cpt++];
	encodedMin[1] = buffer[cpt++];
	min = cl::u8Array_to_i16( encodedMin );

	cl::u8Array<2> encodedMax;
	encodedMax[0] = buffer[cpt++];
	encodedMax[1] = buffer[cpt++];
	max = cl::u8Array_to_i16( encodedMax );

	cl::u8Array<2> encodedStep;
	encodedStep[0] = buffer[cpt++];
	encodedStep[1] = buffer[cpt++];
	step = cl::u8Array_to_i16( encodedStep );

	cl::u8Array<2> encodedDefault;
	encodedDefault[0] = buffer[cpt++];
	encodedDefault[1] = buffer[cpt++];
	defaultValue = cl::u8Array_to_i16( encodedDefault );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		unit[ i ] = static_cast<char>( buffer[ cpt++ ] );
	}
}

