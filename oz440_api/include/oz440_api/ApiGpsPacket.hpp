#ifndef OZCORE_APIGPSPACKET_HPP
#define OZCORE_APIGPSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiGpsPacket : public BaseNaio01Packet
{
public:

	enum GpsType : uint8_t
	{
		RAW = 0x00,
		PARTICLE_FILTER = 0x01,
	};

public:
	ApiGpsPacket( );
	ApiGpsPacket( GpsType gpsType_, ulong time_, double lat_, double lon_, double alt_, uint8_t unit_,	uint8_t satUsed_, uint8_t quality_,	double groundSpeed_ );

	~ApiGpsPacket( );

	virtual cl::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_GPS );
	}

public:
	GpsType gpsType;
	ulong time;
	double lat;
	double lon;
	double alt;
	uint8_t unit;
	uint8_t satUsed;
	uint8_t quality;
	double groundSpeed;
};

typedef std::shared_ptr<ApiGpsPacket> ApiGpsPacketPtr;

#endif //OZCORE_APIGPSPACKET_HPP