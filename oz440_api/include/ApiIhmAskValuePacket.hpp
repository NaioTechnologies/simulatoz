#ifndef OZCORE_APIIHMASKVALUEPACKET_HPP
#define OZCORE_APIIHMASKVALUEPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiIhmAskValuePacket : public BaseNaio01Packet
{
public:
	ApiIhmAskValuePacket( );
	ApiIhmAskValuePacket( uint8_t id, char topLine[20], char question[20], int16_t min, int16_t max, int16_t step, int16_t defaultValue, char unit[20] );
	~ApiIhmAskValuePacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_IHM_ASK_VALUE );
	}

public:
	uint8_t id;
	char topLine[20];
	char question[20];
	int16_t min;
	int16_t max;
	int16_t step;
	int16_t defaultValue;
	char unit[20];
};

typedef std::shared_ptr<ApiIhmAskValuePacket> ApiIhmAskValuePacketPtr;

#endif //OZCORE_APIIHMASKVALUEPACKET_HPP
