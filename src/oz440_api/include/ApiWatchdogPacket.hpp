#ifndef OZCORE_APIWATCHDOGPACKET_HPP
#define OZCORE_APIWATCHDOGPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiWatchdogPacket : public BaseNaio01Packet
{
public:
	ApiWatchdogPacket( uint8_t fooValue_ );
	ApiWatchdogPacket( );
	~ApiWatchdogPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_WATCHDOG );
	}

public:
	uint8_t fooValue;
};

typedef std::shared_ptr<ApiWatchdogPacket> ApiWatchdogPacketPtr;

#endif //OZCORE_APIWATCHDOGPACKET_HPP
