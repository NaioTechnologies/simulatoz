#ifndef OZCORE_HALEDPACKET_HPP
#define OZCORE_HALEDPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaLedPacket : public BaseNaio01Packet
{
public:
	HaLedPacket( );
	HaLedPacket( uint8_t led );
	~HaLedPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_LED );
	}

public:
	uint8_t led;
};

typedef std::shared_ptr<HaLedPacket> HaLedPacketPtr;

#endif //OZCORE_HALEDPACKET_HPP
