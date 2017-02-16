#include "ApiRunPlotPacket.hpp"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
ApiRunPlotPacket::ApiRunPlotPacket( )
{

}

//=============================================================================
//
ApiRunPlotPacket::ApiRunPlotPacket(
		uint16_t rowCount_,
		float rowWidth_[],
		float nextRowDistance_[],
		uint16_t rowLength_[],
		FollowedSide followedSide_,
		FirstNextRowDirection firstNextRowDirection_,
		float shiftDistance_,
		bool exteriorLines_[],
		uint16_t maxSpeed_,
		float cultureWidth_,
		PostOption postOption_,
		DetectorType detectorType_,
		WorkType workType_,
		PassageType passageType_ )
{
	rowCount = rowCount_;

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		rowWidth[i] = rowWidth_[i];
	}

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		nextRowDistance[i] = nextRowDistance_[i];
	}

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		rowLength[i] = rowLength_[i];
	}

	followedSide = followedSide_;
	firstNextRowDirection = firstNextRowDirection_;
	shiftDistance = shiftDistance_;

	exteriorLines[0] = exteriorLines_[0];
	exteriorLines[1] = exteriorLines_[1];

	maxSpeed = maxSpeed_;

	cultureWidth = cultureWidth_;

	postOption = postOption_;

	detectorType = detectorType_;
	workType = workType_;
	passageType = passageType_;

}

//=============================================================================
//
ApiRunPlotPacket::~ApiRunPlotPacket( )
{

}

//=============================================================================
//
cl_copy::BufferUPtr ApiRunPlotPacket::encode()
{
	uint cpt = 0;

	cl_copy::BufferUPtr buffer = cl_copy::unique_buffer( 2 + ( ( 4 + 4 + 2 ) * rowCount ) + 1 + 1 + 4 + 2 + 2 + 4 + 1 + 1 + 1 + 1 );

	cl::u8Array< 2 > encodedRowCount = cl::u16_to_u8Array( rowCount );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowCount[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowCount[ 1 ] );

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		cl::u8Array< 4 > encodedRowWidth = cl::float_to_u8Array( rowWidth[i] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowWidth[ 0 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowWidth[ 1 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowWidth[ 2 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowWidth[ 3 ] );
	}

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		cl::u8Array< 4 > encodedRowDistance = cl::float_to_u8Array( nextRowDistance[i] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowDistance[ 0 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowDistance[ 1 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowDistance[ 2 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowDistance[ 3 ] );
	}

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		cl::u8Array< 2 > encodedRowLength = cl::u16_to_u8Array( rowLength[i] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowLength[ 0 ] );
		(*buffer)[cpt++] = static_cast<uint8_t>( encodedRowLength[ 1 ] );
	}

	(*buffer)[cpt++] = static_cast<uint8_t>( followedSide );

	(*buffer)[cpt++] = static_cast<uint8_t>( firstNextRowDirection );

	cl::u8Array< 4 > encodedShiftDistance = cl::float_to_u8Array( shiftDistance );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedShiftDistance[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedShiftDistance[ 1 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedShiftDistance[ 2 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedShiftDistance[ 3 ] );

	(*buffer)[cpt++] = static_cast<uint8_t>( exteriorLines[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( exteriorLines[ 1 ] );

	cl::u8Array< 2 > encodedMaxSpeed = cl::u16_to_u8Array( maxSpeed );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedMaxSpeed[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedMaxSpeed[ 1 ] );

	cl::u8Array< 4 > encodedCultureWidth = cl::float_to_u8Array( cultureWidth );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedCultureWidth[ 0 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedCultureWidth[ 1 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedCultureWidth[ 2 ] );
	(*buffer)[cpt++] = static_cast<uint8_t>( encodedCultureWidth[ 3 ] );

	(*buffer)[cpt++] = static_cast<uint8_t>( postOption );

	(*buffer)[cpt++] = static_cast<uint8_t>( detectorType );

	(*buffer)[cpt++] = static_cast<uint8_t>( workType );

	(*buffer)[cpt++] = static_cast<uint8_t>( passageType );

	return std::move( getPreparedBuffer( std::move( buffer ), getPacketId() ) );
}

//=============================================================================
//
void ApiRunPlotPacket::decode( uint8_t *buffer, uint bufferSize )
{
	util_copy::ignore( bufferSize );

	uint cpt = getStartPayloadIndex();

	cl::u8Array<2> encodedRowCount;
	encodedRowCount[0] = buffer[cpt++];
	encodedRowCount[1] = buffer[cpt++];
	rowCount = cl::u8Array_to_u16( encodedRowCount );

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		cl::u8Array<4> encodedRowWidth;
		encodedRowWidth[0] = buffer[cpt++];
		encodedRowWidth[1] = buffer[cpt++];
		encodedRowWidth[2] = buffer[cpt++];
		encodedRowWidth[3] = buffer[cpt++];
		rowWidth[i] = cl::u8Array_to_float( encodedRowWidth );
	}

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		cl::u8Array<4> encodedNextRowDistance;
		encodedNextRowDistance[0] = buffer[cpt++];
		encodedNextRowDistance[1] = buffer[cpt++];
		encodedNextRowDistance[2] = buffer[cpt++];
		encodedNextRowDistance[3] = buffer[cpt++];
		nextRowDistance[i] = cl::u8Array_to_float( encodedNextRowDistance );
	}

	for( uint i = 0 ; i < rowCount ; i++ )
	{
		cl::u8Array<2> encodedRowLength;
		encodedRowLength[0] = buffer[cpt++];
		encodedRowLength[1] = buffer[cpt++];
		rowLength[i] = cl::u8Array_to_u16( encodedRowLength );
	}

	followedSide = static_cast<FollowedSide>( buffer[cpt++] );

	firstNextRowDirection = static_cast<FirstNextRowDirection>( buffer[cpt++] );

	cl::u8Array<4> encodedShiftDistance;
	encodedShiftDistance[0] = buffer[cpt++];
	encodedShiftDistance[1] = buffer[cpt++];
	encodedShiftDistance[2] = buffer[cpt++];
	encodedShiftDistance[3] = buffer[cpt++];
	shiftDistance = cl::u8Array_to_float( encodedShiftDistance );

	exteriorLines[0] = static_cast<bool>( buffer[cpt++] );
	exteriorLines[1] = static_cast<bool>( buffer[cpt++] );

	cl::u8Array<2> encodedMaxSpeed;
	encodedMaxSpeed[0] = buffer[cpt++];
	encodedMaxSpeed[1] = buffer[cpt++];
	maxSpeed = cl::u8Array_to_u16( encodedMaxSpeed );

	cl::u8Array<4> encodedCultureWidth;
	encodedCultureWidth[0] = buffer[cpt++];
	encodedCultureWidth[1] = buffer[cpt++];
	encodedCultureWidth[2] = buffer[cpt++];
	encodedCultureWidth[3] = buffer[cpt++];
	cultureWidth = cl::u8Array_to_float( encodedCultureWidth );

	postOption = static_cast<PostOption>( buffer[cpt++] );

	detectorType = static_cast<DetectorType>( buffer[cpt++] );

	workType = static_cast<WorkType>( buffer[cpt++] );

	passageType = static_cast<PassageType>( buffer[cpt++] );
}

