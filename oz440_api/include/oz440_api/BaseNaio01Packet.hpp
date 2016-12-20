#ifndef OZCORE_BASENAIO01PACKET_HPP
#define OZCORE_BASENAIO01PACKET_HPP

#include "CLBuffer.hpp"

class BaseNaio01Packet
{
	public:

	BaseNaio01Packet();
	virtual ~BaseNaio01Packet();

	virtual cl::BufferUPtr encode() = 0;

	virtual void decode( uint8_t *buffer, uint bufferSize ) = 0;

	virtual uint8_t getPacketId() = 0;

	cl::BufferUPtr getPreparedBuffer( cl::BufferUPtr buffer, const uint8_t packetId );

	protected:

	uint getStartPayloadIndex();

	template< class... Args >
	void ignore( Args&& ... )
	{ }

	private:

	uint startPayloadIndex = 11;
};

typedef std::shared_ptr<BaseNaio01Packet> BaseNaio01PacketPtr;

#endif //OZCORE_BASENAIO01PACKET_HPP
