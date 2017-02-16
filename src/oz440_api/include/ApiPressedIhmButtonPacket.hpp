#ifndef OZCORE_APIPRESSEDIHMBUTTONPACKET_HPP
#define OZCORE_APIPRESSEDIHMBUTTONPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiPressedIhmButtonPacket : public BaseNaio01Packet
{
public:
	typedef enum PressedIhmButtonIndex_ {
		BI_VALID = 1,
		BI_CANCEL = 0,
		BI_NEXT = 4,
		BI_PREV = 5,
		BI_PLUS = 2,
		BI_MOINS = 3,
	} PressedIhmButtonIndex;
public:
	ApiPressedIhmButtonPacket( );
	ApiPressedIhmButtonPacket( PressedIhmButtonIndex pressedIhmButton_ );
	~ApiPressedIhmButtonPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_PRESSED_IHM_BUTTON );
	}

public:
	PressedIhmButtonIndex pressedIhmButton;
};

typedef std::shared_ptr<ApiPressedIhmButtonPacket> ApiPressedIhmButtonPacketPtr;

#endif //OZCORE_APIPRESSEDIHMBUTTONPACKET_HPP
