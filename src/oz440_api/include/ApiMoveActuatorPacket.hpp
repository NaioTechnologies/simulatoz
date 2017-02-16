#ifndef OZCORE_APIMOVEACTUATORPACKET_HPP
#define OZCORE_APIMOVEACTUATORPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiMoveActuatorPacket : public BaseNaio01Packet
{
public:
	ApiMoveActuatorPacket( );
	ApiMoveActuatorPacket( uint8_t position_ );
	~ApiMoveActuatorPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_MOVE_ACTUATOR );
	}

public:
	uint8_t position;
};

typedef std::shared_ptr<ApiMoveActuatorPacket> ApiMoveActuatorPacketPtr;

#endif //OZCORE_APIMOVEACTUATORPACKET_HPP
