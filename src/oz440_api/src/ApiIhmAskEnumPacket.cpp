#include "ApiIhmAskEnumPacket.hpp"
#include "vitals/CLByteConversion.h"
#include <string.h>

//=============================================================================
//
ApiIhmAskEnumPacket::ApiIhmAskEnumPacket( )
{

}

//=============================================================================
//
ApiIhmAskEnumPacket::ApiIhmAskEnumPacket( uint8_t id_, char topLine_[20], char question_[20], uint8_t optionCount_, char option_[20][20], uint8_t defaultOption_, char unit_[20] )
{
	id = id_;

	memcpy( topLine, topLine_, 20 );

	memcpy( question, question_, 20 );

	optionCount = optionCount_;

	for( uint i = 0 ; i < 20 ; i ++ )
	{
		memcpy( option[i], option_[i], 20 );
	}

	defaultOption = defaultOption_;

	memcpy( unit, unit_, 20 );
}

//=============================================================================
//
ApiIhmAskEnumPacket::~ApiIhmAskEnumPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiIhmAskEnumPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 1 + 20 + 20 + 1 + (20*20) + 1 + 20 );

	(*buffer)[cpt++] = static_cast<uint8_t>( id );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( topLine[ i ] );
	}

	for( uint i = 0 ; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( question[ i ] );
	}

	(*buffer)[cpt++] = static_cast<uint8_t>( optionCount );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		for (uint j = 0; j < 20; j++)
		{
			(*buffer)[cpt++] = static_cast<uint8_t>( option[i][j] );
		}
	}

	(*buffer)[cpt++] = static_cast<uint8_t>( defaultOption );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		(*buffer)[cpt++] = static_cast<uint8_t>( unit[ i ] );
	}

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiIhmAskEnumPacket::decode( uint8_t *buffer, uint bufferSize )
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

	optionCount =  static_cast<uint8_t>( buffer[ cpt++ ] );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		for (uint j = 0; j < 20; j++)
		{
			option[i][j] = static_cast<char>( buffer[ cpt++ ] );
		}
	}

	defaultOption =  static_cast<uint8_t>( buffer[ cpt++ ] );

	for( uint i = 0 ; i < 20 ; i++ )
	{
		unit[ i ] = static_cast<char>( buffer[ cpt++ ] );
	}
}

