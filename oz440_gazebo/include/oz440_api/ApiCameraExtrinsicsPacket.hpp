#ifndef OZCORE_APICAMERAEXTRINSICSPACKET_HPP
#define OZCORE_APICAMERAEXTRINSICSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiCameraExtrinsicsPacket : public BaseNaio01Packet
{
public:
	ApiCameraExtrinsicsPacket( );
	ApiCameraExtrinsicsPacket( double mat_[], double vec_[] );

	~ApiCameraExtrinsicsPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_CAMERA_EXTRINSICS );
	}

public:
	double mat[9];
	double vec[3];
};

typedef std::shared_ptr<ApiCameraExtrinsicsPacket> ApiCameraExtrinsicsPacketPtr;

#endif //OZCORE_APICAMERAINTRINSICSPACKET_HPP
