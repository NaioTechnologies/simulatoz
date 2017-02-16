#ifndef OZCORE_APISTATUSPACKET_HPP
#define OZCORE_APISTATUSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiStatusPacket : public BaseNaio01Packet
{
public:
	ApiStatusPacket( );
	ApiStatusPacket( bool imuRested_, double theta_, int32_t odoFR_, int32_t odoRR_, int32_t odoRL_, int32_t odoFL_, double positionX_, double positionY_, double distance_, uint8_t actuatorPosition_, uint8_t battery_,
					 int16_t magX_, int16_t magY_, int16_t magZ_ );
	~ApiStatusPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_STATUS );
	}

public:
	bool imuReseted;

	double theta;

	int32_t odoFR;
	int32_t odoRR;
	int32_t odoRL;
	int32_t odoFL;

	double positionX;
	double positionY;

	double distance;

	uint8_t actuatorPosition;
	uint8_t battery;

	int16_t magX;
	int16_t magY;
	int16_t magZ;
};

typedef std::shared_ptr<ApiStatusPacket> ApiStatusPacketPtr;

#endif //OZCORE_STATUSPACKET_HPP
