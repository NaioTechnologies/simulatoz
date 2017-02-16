#ifndef OZCORE_BASENAIO01PACKET_HPP
#define OZCORE_BASENAIO01PACKET_HPP

#include "vitals/CLBuffer.hpp"

class BaseNaio01Packet
{
	public:

	BaseNaio01Packet();
	virtual ~BaseNaio01Packet();

	virtual cl_copy::BufferUPtr encode() = 0;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) = 0;

	virtual uint8_t getPacketId() = 0;

	cl_copy::BufferUPtr getPreparedBuffer( cl_copy::BufferUPtr buffer, const uint8_t packetId );

	protected:

	uint32_t getStartPayloadIndex();

	private:

	uint32_t startPayloadIndex = 11;
};

typedef std::shared_ptr<BaseNaio01Packet> BaseNaio01PacketPtr;

#endif //OZCORE_BASENAIO01PACKET_HPP
