#ifndef OZCORE_APILIDARPACKET_HPP
#define OZCORE_APILIDARPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiLidarPacket : public BaseNaio01Packet
{
public:
	ApiLidarPacket( );
	ApiLidarPacket( uint16_t distance_[271] );
	~ApiLidarPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_LIDAR );
	}

public:
	uint16_t distance[271];
};

typedef std::shared_ptr<ApiLidarPacket> ApiLidarPacketPtr;

#endif //OZCORE_APILIDARPACKET_HPP
