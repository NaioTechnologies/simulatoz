#ifndef OZCORE_APIRUNPLOTPACKET_HPP
#define OZCORE_APIRUNPLOTPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"
#include "../../../../../../../usr/include/stdint.h"

#define APIRUNPLOTPACKET_MAX_ROWS 100

class ApiRunPlotPacket : public BaseNaio01Packet
{
public:
	enum FirstNextRowDirection : uint8_t
	{
		FNRD_LEFT = 0x00,
		FNRD_RIGHT = 0x01,
	};

	enum FollowedSide : uint8_t
	{
		FS_LEFT = 0x00,
		FS_CENTER = 0x01,
		FS_RIGHT = 0x02
	};

	enum PostOption : uint8_t
	{
		PO_NO_POST = 0x00,
		PO_USE_POST = 0x01,
		PO_USE_POST_SURE = 0x02,
	};

	enum DetectorType : uint8_t
	{
		DT_LIDAR = 0x00,
		DT_CAMERA = 0x01,
		DT_PLASTIC_MULCH = 0x02,
	};

	enum WorkType : uint8_t
	{
		WT_BINAGE = 0x00,
		WT_PARCOURS = 0x01
	};

	enum PassageType : uint8_t
	{
		PT_ONE_PASSAGE_MAX = 0x00,
		PT_TWO_PASSAGE_MAX = 0x01,
		PT_THREE_PASSAGE_MAX = 0x02
	};

public:
	ApiRunPlotPacket( );
	ApiRunPlotPacket(
			uint16_t rowCount_,
			float rowWidth_[],
			float nextRowDistance_[],
			uint16_t rowLength_[],
			FollowedSide followedSide_,
			FirstNextRowDirection firstNextRowDirection_,
			float shiftDistance_,
			bool exteriorLines_[],
			uint16_t maxSpeed_,
			float cultureWidth_,
			PostOption postOption_,
			DetectorType detectorType_,
			WorkType workType_,
			PassageType passageType_ );

	~ApiRunPlotPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_RUN_PLOT_VALUE );
	}

public:
	uint16_t rowCount;
	float rowWidth[APIRUNPLOTPACKET_MAX_ROWS];
	float nextRowDistance[APIRUNPLOTPACKET_MAX_ROWS];
	uint16_t rowLength[APIRUNPLOTPACKET_MAX_ROWS];

	FollowedSide followedSide;
	FirstNextRowDirection firstNextRowDirection;

	float shiftDistance;
	bool exteriorLines[2];

	uint16_t maxSpeed;
	float cultureWidth;

	PostOption postOption;
	DetectorType detectorType;
	WorkType workType;
	PassageType passageType;
};

typedef std::shared_ptr<ApiRunPlotPacket> ApiRunPlotPacketPtr;

#endif //OZCORE_APIRUNPLOTPACKET_HPP
