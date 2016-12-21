#ifndef OZCORE_HAMAGNETOPACKET_HPP
#define OZCORE_HAMAGNETOPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaMagnetoPacket : public BaseNaio01Packet
{
public:
	HaMagnetoPacket( );
	HaMagnetoPacket( int16_t x, int16_t y, int16_t z );
	~HaMagnetoPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_MAGNETO );
	}

public:
	int16_t x;
	int16_t y;
	int16_t z;
};

typedef std::shared_ptr<HaMagnetoPacket> HaMagnetoPacketPtr;

#endif //OZCORE_HAMAGNETOPACKET_HPP
