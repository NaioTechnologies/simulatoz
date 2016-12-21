#ifndef OZCORE_APICAMERAINTRINSICSPACKET_HPP
#define OZCORE_APICAMERAINTRINSICSPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"

class ApiCameraIntrinsicsPacket : public BaseNaio01Packet
{
public:
	enum SourceCamera : uint8_t
	{
		LEFT = 0x00,
		RIGHT = 0x01,
	};

	ApiCameraIntrinsicsPacket( );
	ApiCameraIntrinsicsPacket(
			SourceCamera sourceCamera_,
			double focalLength_,     // Focal length for fisheye correction
			double principalPointX_, // Principal point X coordinate
			double principalPointY_, // Principal point Y coordinate
			double fX_,              // Focal length X axis
			double fY_,             // Focal length Y axis
			double k_1_,             // 1st radial distortion coefficient
			double k_2_,             // 2nd radial distortion coefficient
			double p_1_,             // 1st tangential distortion coefficient
			double p_2_,             // 2nd tangential distortion coefficient
			double k_3_,             // 3rd radial distortion coefficient
			double k_4_,             // 4th radial distortion coefficient
			double k_5_,             // 5th radial distortion coefficient
			double k_6_             // 6th radial distortion coefficient
	);

	~ApiCameraIntrinsicsPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_CAMERA_INTRINSICS );
	}

public:
	SourceCamera sourceCamera;
	double focalLength;     // Focal length for fisheye correction
	double principalPointX; // Principal point X coordinate
	double principalPointY; // Principal point Y coordinate
	double fX;              // Focal length X axis
	double fY;              // Focal length Y axis
	double k_1;             // 1st radial distortion coefficient
	double k_2;             // 2nd radial distortion coefficient
	double p_1;             // 1st tangential distortion coefficient
	double p_2;             // 2nd tangential distortion coefficient
	double k_3;             // 3rd radial distortion coefficient
	double k_4;             // 4th radial distortion coefficient
	double k_5;             // 5th radial distortion coefficient
	double k_6;             // 6th radial distortion coefficient
};

typedef std::shared_ptr<ApiCameraIntrinsicsPacket> ApiCameraIntrinsicsPacketPtr;

#endif //OZCORE_APICAMERAINTRINSICSPACKET_HPP
