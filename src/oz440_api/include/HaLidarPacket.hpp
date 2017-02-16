#ifndef OZCORE_HALIDARPACKET_HPP
#define OZCORE_HALIDARPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class HaLidarPacket : public BaseNaio01Packet
{
public:
	HaLidarPacket( );
	HaLidarPacket( uint16_t distance[271], uint8_t albedo[271] );
	~HaLidarPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_LIDAR );
	}

public:
	uint16_t distance[271];
	uint8_t albedo[271];
};

typedef std::shared_ptr<HaLidarPacket> HaLidarPacketPtr;

#endif //OZCORE_HALIDARPACKET_HPP
