#ifndef OZCORE_HAACTUATORPACKET_HPP
#define OZCORE_HAACTUATORPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaActuatorPacket : public BaseNaio01Packet
{
public:
	HaActuatorPacket( );
	HaActuatorPacket( bool isRequest, bool isPosition, uint8_t position );
	~HaActuatorPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_ACTUATOR );
	}

public:
	bool isRequest;
	bool isPosition;
	uint8_t position;
};

typedef std::shared_ptr<HaActuatorPacket> HaActuatorPacketPtr;

#endif //OZCORE_HAACTUATORPACKET_HPP
