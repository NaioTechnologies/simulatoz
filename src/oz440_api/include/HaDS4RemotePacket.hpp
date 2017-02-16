#ifndef OZCORE_DS4REMOTEPACKET_HPP
#define OZCORE_DS4REMOTEPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaDS4RemotePacket : public BaseNaio01Packet
{
public:
	HaDS4RemotePacket( );
	HaDS4RemotePacket( uint8_t battery,	int8_t leftAnalogX,	int8_t leftAnalogY,	int8_t rightAnalogX, int8_t rightAnalogY,
					   uint8_t buttons, uint8_t l1l3r1r3, uint8_t l2, uint8_t r2, int8_t accelX, int8_t accelY, int8_t accelZ,
					   int8_t gyroX, int8_t gyroY, int8_t gyroZ );
	~HaDS4RemotePacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_DS4REMOTE );
	}

public:
	uint8_t battery;

	int8_t leftAnalogX;
	int8_t leftAnalogY;
	int8_t rightAnalogX;
	int8_t rightAnalogY;

	uint8_t buttons;
	uint8_t l1l3r1r3;

	uint8_t l2;
	uint8_t r2;

	int8_t accelX;
	int8_t accelY;
	int8_t accelZ;

	int8_t gyroX;
	int8_t gyroY;
	int8_t gyroZ;
};

typedef std::shared_ptr<HaDS4RemotePacket> HaDS4RemotePacketPtr;

#endif //OZCORE_DS4REMOTEPACKET_HPP
