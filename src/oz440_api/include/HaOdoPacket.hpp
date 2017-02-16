#ifndef OZCORE_HAODOPACKET_HPP
#define OZCORE_HAODOPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaOdoPacket : public BaseNaio01Packet
{
public:
	HaOdoPacket( );
	HaOdoPacket( uint8_t fr, uint8_t rr, uint8_t rl, uint8_t fl );
	~HaOdoPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_ODO );
	}

public:
	uint8_t fr;
	uint8_t rr;
	uint8_t rl;
	uint8_t fl;
};

typedef std::shared_ptr<HaOdoPacket> HaOdoPacketPtr;

#endif //OZCORE_HAODOPACKET_HPP
