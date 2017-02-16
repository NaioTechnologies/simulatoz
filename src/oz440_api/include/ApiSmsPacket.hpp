#ifndef OZCORE_APISMSPACKET_HPP
#define OZCORE_APISMSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiSmsPacket : public BaseNaio01Packet
{
public:

	enum SmsType : uint8_t
	{
		TO_SEND = 0x00,
		RECEIVED = 0x01,
	};

public:
	ApiSmsPacket( );
	ApiSmsPacket( SmsType smsType_, std::string recipient_, std::string message_ );

	~ApiSmsPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_SMS );
	}

public:
	SmsType smsType;
	std::string recipient;
	std::string message;
};

typedef std::shared_ptr<ApiSmsPacket> ApiSmsPacketPtr;

#endif //OZCORE_APISMSPACKET_HPP
