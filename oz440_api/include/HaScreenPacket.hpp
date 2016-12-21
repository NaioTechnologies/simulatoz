#ifndef OZCORE_HASCREENPACKET_HPP
#define OZCORE_HASCREENPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaScreenPacket : public BaseNaio01Packet
{
public:
	HaScreenPacket( );
	HaScreenPacket( char topLine_[16], char bottomLine_[16] );
	~HaScreenPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_SCREEN );
	}

public:
	char topLine[16];
	char bottomLine[16];
};

typedef std::shared_ptr<HaScreenPacket> HaScreenPacketPtr;

#endif //OZCORE_HASCREENPACKET_HPP
