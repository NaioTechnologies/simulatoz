#ifndef OZCORE_HASTEREOCAMERAPACKET_HPP
#define OZCORE_HASTEREOCAMERAPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

#define BUFFER_SIZE 541440

class HaStereoCameraPacket : public BaseNaio01Packet
{
public:
	HaStereoCameraPacket( );
	HaStereoCameraPacket( uint8_t buffer_[BUFFER_SIZE]);
	~HaStereoCameraPacket( );

	virtual cl::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::HA_RAW_STEREO_CAMERA );
	}

public:
	cl::BufferSPtr dataBuffer;
};

typedef std::shared_ptr<HaStereoCameraPacket> HaStereoCameraPacketPtr;

#endif //OZCORE_HASTEREOCAMERAPACKET_HPP