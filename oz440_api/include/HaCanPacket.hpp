#ifndef OZCORE_HACANPACKET_HPP
#define OZCORE_HACANPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaCanPacket : public BaseNaio01Packet
{
public:
	HaCanPacket( );
	HaCanPacket( uint8_t dataBuffer[21], uint8_t dataBufferSize );
	~HaCanPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_CAN );
	}

public:
	uint8_t dataBufferSize;
	uint8_t dataBuffer[21];
};

typedef std::shared_ptr<HaCanPacket> HaCanPacketPtr;

#endif //OZCORE_HACANPACKET_HPP
