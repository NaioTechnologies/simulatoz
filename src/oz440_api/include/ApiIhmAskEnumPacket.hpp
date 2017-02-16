#ifndef OZCORE_APIIHMASKENUMPACKET_HPP
#define OZCORE_APIIHMASKENUMPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiIhmAskEnumPacket : public BaseNaio01Packet
{
public:
	ApiIhmAskEnumPacket( );
	ApiIhmAskEnumPacket( uint8_t id,	char topLine[20], char question[20], uint8_t optionCount, char option[20][20], uint8_t defaultOption, char unit[20] );
	~ApiIhmAskEnumPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_IHM_ASK_ENUM );
	}

public:
	uint8_t id;

	char topLine[20];
	char question[20];

	uint8_t optionCount;
	char option[20][20];
	uint8_t defaultOption;

	char unit[20];
};

typedef std::shared_ptr<ApiIhmAskEnumPacket> ApiIhmAskEnumPacketPtr;

#endif //OZCORE_APIIHMASKENUMPACKET_HPP
