#ifndef OZCORE_ENUMRESPONSEPACKET_HPP
#define OZCORE_ENUMRESPONSEPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiEnumResponsePacket : public BaseNaio01Packet
{
public:
	enum KeyPressedType : uint8_t
	{
		VALIDATE = 0x00,
		CANCEL = 0x01,
		LEFT = 0x02,
		RIGHT = 0x03,
		MINUS = 0x04,
		PLUS = 0x05
	};

public:
	ApiEnumResponsePacket( );
	ApiEnumResponsePacket( uint8_t id_, KeyPressedType keyPressedType_, uint8_t selectedOption_ );
	~ApiEnumResponsePacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_ENUM_RESPONSE );
	}

public:
	uint8_t id;
	KeyPressedType keyPressedType;
	uint8_t selectedOption;
};

typedef std::shared_ptr<ApiEnumResponsePacket> ApiEnumResponsePacketPtr;

#endif //OZCORE_ENUMRESPONSEPACKET_HPP
