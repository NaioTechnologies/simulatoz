#ifndef OZCORE_HACCELROPACKET_HPP
#define OZCORE_HACCELROPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaAcceleroPacket : public BaseNaio01Packet
{
public:
	HaAcceleroPacket( );
	HaAcceleroPacket( int16_t x, int16_t y, int16_t z );
	~HaAcceleroPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_ACCELERO );
	}

public:
	int16_t x;
	int16_t y;
	int16_t z;
};

typedef std::shared_ptr<HaAcceleroPacket> HaAcceleroPacketPtr;

#endif //OZCORE_HACCELROPACKET_HPP
