#ifndef OZCORE_APILOGTOROBOTPACKET_HPP
#define OZCORE_APILOGTOROBOTPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiLogToRobotPacket : public BaseNaio01Packet
{
public:
public:
	ApiLogToRobotPacket( );
	ApiLogToRobotPacket( std::string message_ );

	~ApiLogToRobotPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_LOG_TO_ROBOT );
	}

public:
	std::string message;
};

typedef std::shared_ptr<ApiLogToRobotPacket> ApiLogToRobotPacketPtr;

#endif //OZCORE_APILOGTOROBOTPACKET_HPP
