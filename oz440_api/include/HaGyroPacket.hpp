#ifndef OZCORE_GYROPACKET_HPP
#define OZCORE_GYROPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaGyroPacket : public BaseNaio01Packet
{
public:
	HaGyroPacket( );
	HaGyroPacket( int16_t x, int16_t y, int16_t z );
	~HaGyroPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_GYRO );
	}

public:
	int16_t x;
	int16_t y;
	int16_t z;
};

typedef std::shared_ptr<HaGyroPacket> HaGyroPacketPtr;

#endif //OZCORE_GYROPACKET_HPP
