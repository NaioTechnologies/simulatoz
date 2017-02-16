#ifndef OZCORE_APICOMMANDPACKET_HPP
#define OZCORE_APICOMMANDPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiCommandPacket : public BaseNaio01Packet
{
public:
	enum CommandType : uint8_t
	{
		RESET_IMU = 0x01,
		TURN_ON_API_RECTIFIED_STEREO_CAMERA_PACKET = 0x02,
		TURN_OFF_API_RECTIFIED_STEREO_CAMERA_PACKET = 0x03,
		GET_CAMERA_INTRINSICS_PACKET = 0x04,
		GET_CAMERA_EXTRINSICS_PACKET = 0x05,
		TURN_ON_API_RAW_STEREO_CAMERA_PACKET = 0x06,
		TURN_OFF_API_RAW_STEREO_CAMERA_PACKET = 0x07,
		TURN_ON_API_UNRECTIFIED_STEREO_CAMERA_PACKET = 0x08,
		TURN_OFF_API_UNRECTIFIED_STEREO_CAMERA_PACKET = 0x09,
		STOP_CURRENT_WORK = 0x0A,
		PAUSE_CURRENT_WORK = 0x0B,
		RESUME_CURRENT_WORK = 0x0C,
		TURN_ON_IMAGE_ZLIB_COMPRESSION = 0x0D,
		TURN_OFF_IMAGE_ZLIB_COMPRESSION = 0x0E,
	};

public:
	ApiCommandPacket( );
	ApiCommandPacket( CommandType commandType_ );
	~ApiCommandPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_COMMAND );
	}

public:
	CommandType commandType;
};

typedef std::shared_ptr<ApiCommandPacket> ApiCommandPacketPtr;

#endif //OZCORE_APICOMMANDPACKET_HPP
