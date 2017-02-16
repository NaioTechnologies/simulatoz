#ifndef OZCORE_HAKEYPADPACKET_HPP
#define OZCORE_HAKEYPADPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaKeypadPacket : public BaseNaio01Packet
{
public:
	HaKeypadPacket( );
	HaKeypadPacket( uint8_t keypad );
	~HaKeypadPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_KEYPAD );
	}

public:
	uint8_t keypad;
};

typedef std::shared_ptr<HaKeypadPacket> HaKeypadPacketPtr;

#endif //OZCORE_HAKEYPADPACKET_HPP
