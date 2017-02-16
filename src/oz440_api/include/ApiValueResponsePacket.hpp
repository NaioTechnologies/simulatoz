#ifndef OZCORE_VALUERESPONSEPACKET_HPP
#define OZCORE_VALUERESPONSEPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiValueResponsePacket : public BaseNaio01Packet
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
	ApiValueResponsePacket( );
	ApiValueResponsePacket( uint8_t id_, KeyPressedType keyPressedType_, int16_t selectedValue_ );
	~ApiValueResponsePacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_VALUE_RESPONSE );
	}

public:
	uint8_t id;
	KeyPressedType keyPressedType;
	int16_t selectedValue;
};

typedef std::shared_ptr<ApiValueResponsePacket> ApiValueResponsePacketPtr;

#endif //OZCORE_VALUERESPONSEPACKET_HPP
