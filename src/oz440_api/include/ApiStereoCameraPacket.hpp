#ifndef OZCORE_APISTEREOCAMERAPACKET_HPP
#define OZCORE_APISTEREOCAMERAPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiStereoCameraPacket : public BaseNaio01Packet
{
public:

	enum ImageType : uint8_t
	{
		RAW_IMAGES = 0x01, //  752 * 480 * 1 (BAYER)
		UNRECTIFIED_COLORIZED_IMAGES = 0x02, //  752 * 480 * 3 (RGB)
		RECTIFIED_COLORIZED_IMAGES = 0x03, // 376 * 240 * 3 (RGB)
		RAW_IMAGES_ZLIB = 0x04, //  752 * 480 * 1 (BAYER)
		UNRECTIFIED_COLORIZED_IMAGES_ZLIB = 0x05, //  752 * 480 * 3 (RGB)
		RECTIFIED_COLORIZED_IMAGES_ZLIB = 0x06, // 376 * 240 * 3 (RGB)
	};

	ApiStereoCameraPacket( );
	ApiStereoCameraPacket( ImageType imageType_, cl_copy::BufferUPtr dataBuffer_ );
	~ApiStereoCameraPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_RAW_STEREO_CAMERA );
	}

public:
	ImageType imageType;
	cl_copy::BufferUPtr dataBuffer;
};

typedef std::shared_ptr<ApiStereoCameraPacket> ApiStereoCameraPacketPtr;

#endif //OZCORE_APISTEREOCAMERAPACKET_HPP
