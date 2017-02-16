#ifndef OZCORE_APIIHMDISPLAYPACKET_HPP
#define OZCORE_APIIHMDISPLAYPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiIhmDisplayPacket : public BaseNaio01Packet
{
public:
	ApiIhmDisplayPacket( );
	ApiIhmDisplayPacket( const char topLine_[20], const char bottomLine_[20] );
	~ApiIhmDisplayPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_IHM_DISPLAY );
	}

public:
	char topLine[20];
	char bottomLine[20];
};

typedef std::shared_ptr<ApiIhmDisplayPacket> ApiIhmDisplayPacketPtr;

#endif //OZCORE_APIIHMDISPLAYPACKET_HPP
