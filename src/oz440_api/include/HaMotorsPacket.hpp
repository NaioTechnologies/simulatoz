#ifndef OZCORE_HAMOTORSPACKET_HPP
#define OZCORE_HAMOTORSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class HaMotorsPacket : public BaseNaio01Packet
{
public:
	HaMotorsPacket( );
	HaMotorsPacket( int8_t left_, int8_t right_ );
	~HaMotorsPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_MOTORS );
	}

public:
	int8_t left;
	int8_t right;
};

typedef std::shared_ptr<HaMotorsPacket> HaMotorsPacketPtr;

#endif //OZCORE_HAMOTORSPACKET_HPP
