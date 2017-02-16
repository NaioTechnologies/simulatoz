#ifndef OZCORE_APIMOTORSPACKET_HPP
#define OZCORE_APIMOTORSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiMotorsPacket : public BaseNaio01Packet
{
public:
	ApiMotorsPacket( );
	ApiMotorsPacket( int8_t left_, int8_t right_ );
	~ApiMotorsPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_MOTORS );
	}

public:
	int8_t left;
	int8_t right;
};

typedef std::shared_ptr<ApiMotorsPacket> ApiMotorsPacketPtr;

#endif //OZCORE_APIMOTORSPACKET_HPP
