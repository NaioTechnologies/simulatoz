#ifndef OZCORE_APIGPRSPACKET_HPP
#define OZCORE_APIGPRSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiGprsPacket : public BaseNaio01Packet
{
public:

	enum GprsCommandeType : uint8_t
	{
		LAST_COMMAND_OK = 0x00,
		LAST_COMMAND_KO = 0x01,
		OPEN_CONNECTION = 0x02,
		SEND_DATA = 0x03,
		DATA_RECEIVED = 0x04,
		CLOSE_CONNECTION = 0x05,
	};

public:
	ApiGprsPacket( );
	ApiGprsPacket( GprsCommandeType gprsCommandeType_ );
	ApiGprsPacket( uint16_t port_, std::string adress_ );
	ApiGprsPacket( GprsCommandeType gprsCommandeType_, uint8_t *buffer, uint32_t bufferSize );

	~ApiGprsPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_GPRS );
	}

public:
	GprsCommandeType gprsCommandeType;
	uint16_t port;
	std::string adress;
	cl_copy::BufferUPtr dataPtr;
};

typedef std::shared_ptr<ApiGprsPacket> ApiGprsPacketPacketPtr;

#endif //OZCORE_APIGPRSPACKET_HPP
