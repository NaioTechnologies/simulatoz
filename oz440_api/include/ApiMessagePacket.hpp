#ifndef OZCORE_APIMESSAGEPACKET_HPP
#define OZCORE_APIMESSAGEPACKET_HPP

#include <vitals/Types.h>
#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiMessagePacket : public BaseNaio01Packet
{
public:
	ApiMessagePacket( );
	ApiMessagePacket( API_MESSAGE apiMessage_ );
	~ApiMessagePacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_MESSAGE );
	}

public:
	API_MESSAGE apiMessage;
};

typedef std::shared_ptr<ApiMessagePacket> ApiMessagePacketPtr;

#endif //OZCORE_APIMESSAGEPACKET_HPP
