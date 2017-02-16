#ifndef OZCORE_HAGPSPACKET_HPP
#define OZCORE_HAGPSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class HaGpsPacket : public BaseNaio01Packet
{
public:
	HaGpsPacket( );
	HaGpsPacket( uint64_t time_, double lat_, double lon_, double alt_, uint8_t unit_,	uint8_t satUsed_, uint8_t quality_,	double groundSpeed_ );

	~HaGpsPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_GPS );
	}

public:
	uint64_t time;
	double lat;
	double lon;
	double alt;
	uint8_t unit;
	uint8_t satUsed;
	uint8_t quality;
	double groundSpeed;
};

typedef std::shared_ptr<HaGpsPacket> HaGpsPacketPtr;

#endif //OZCORE_HAGPSPACKET_HPP
