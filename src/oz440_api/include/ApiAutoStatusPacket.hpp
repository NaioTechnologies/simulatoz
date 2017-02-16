#ifndef OZCORE_APIAUTOSTATUSPACKET_HPP
#define OZCORE_APIAUTOSTATUSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiAutoStatusPacket : public BaseNaio01Packet
{
public:
	enum AutoStatusType : uint8_t
	{
		WATCHDOG = 0x00,
		BINAGE_START = 0x01,
		BINAGE_PAUSE = 0x02,
		BINAGE_WAITING_USER_INPUT = 0x03,
		BINAGE_PAUSE_RESUME = 0x04,
		PROBLEM_OBSTACLE_DETECTED = 0x05,
		PROBLEM_HOLE_IN_LINE = 0x06,
		PROBLEM_LINE_TOO_LONG = 0x07,
		PROBLEM_EXIT_NOT_DETECTED = 0x08,
		PROBLEM_ENTRANCE_NOT_DETECTED = 0x09,
		HARDWARE_TROUBLE = 0xA0,
	};

public:
	ApiAutoStatusPacket( );
	ApiAutoStatusPacket( AutoStatusType autoStatusType_ );
	~ApiAutoStatusPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_AUTO_STATUS );
	}

public:
	AutoStatusType autoStatusType;
};

typedef std::shared_ptr<ApiAutoStatusPacket> ApiAutoStatusPacketPtr;

#endif //OZCORE_APIAUTOSTATUSPACKET_HPP
