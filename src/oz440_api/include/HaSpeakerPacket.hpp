#ifndef OZCORE_HASPEAKERPACKET_HPP
#define OZCORE_HASPEAKERPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaSpeakerPacket : public BaseNaio01Packet
{
public:
	HaSpeakerPacket( );
	HaSpeakerPacket( uint8_t duration, uint8_t volume );
	~HaSpeakerPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_SPEAKER );
	}

public:
	uint8_t duration;
	uint8_t volume;
};

typedef std::shared_ptr<HaSpeakerPacket> HaSpeakerPacketPtr;

#endif //OZCORE_HASPEAKERPACKET_HPP
