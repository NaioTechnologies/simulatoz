#include "BaseNaio01Packet.hpp"
#include "vitals/CLArray.h"
#include "vitals/CLByteConversion.h"

//=============================================================================
//
BaseNaio01Packet::BaseNaio01Packet()
{

}

//=============================================================================
//
BaseNaio01Packet::~BaseNaio01Packet()
{

}

//=============================================================================
//
uint BaseNaio01Packet::getStartPayloadIndex()
{
	return startPayloadIndex;
}

//=============================================================================
//
cl_copy::BufferUPtr BaseNaio01Packet::getPreparedBuffer( cl_copy::BufferUPtr buffer, const uint8_t packetId )
{
	int packetSize = 6 + 1 + 4;
	cl_copy::BufferUPtr preparedBuffer = cl_copy::unique_buffer( packetSize );

	// HEADER
	(*preparedBuffer)[0] = 0x4e;
	(*preparedBuffer)[1] = 0x41;
	(*preparedBuffer)[2] = 0x49;
	(*preparedBuffer)[3] = 0x4f;
	(*preparedBuffer)[4] = 0x30;
	(*preparedBuffer)[5] = 0x31;

	(*preparedBuffer)[6] = packetId;

	// Add the size of the message to the packet
	cl::u8Array< 4 > byteArr = cl::u32_to_u8Array( static_cast<uint32_t>( buffer->size() ) );
	(*preparedBuffer)[7] = byteArr.at( 0 );
	(*preparedBuffer)[8] = byteArr.at( 1 );
	(*preparedBuffer)[9] = byteArr.at( 2 );
	(*preparedBuffer)[10] = byteArr.at( 3 );

	// PAYLOAD
	preparedBuffer->insert( std::move( buffer ), startPayloadIndex );

	// CRC
	uint8_t checksum[4]{ 0, 0, 0, 0 };
	preparedBuffer->append( checksum, 4 );

	return std::move( preparedBuffer );
}
