#ifndef OZCORE_NAIO01CODEC_HPP
#define OZCORE_NAIO01CODEC_HPP

#include <memory>
#include <vector>
#include "vitals/CLBuffer.hpp"
#include "BaseNaio01Packet.hpp"

class Naio01Codec
{
	public:

	enum class Naio01CodecPacketType : uint8_t
	{
		HA_MOTORS = 0x01,
		HA_GPS = 0x04,
		HA_ODO = 0x06,
		HA_LIDAR = 0x07,
		HA_DS4REMOTE = 0x08,
		HA_ACCELERO = 0x09,
		HA_ACTUATOR = 0x0F,
		HA_GYRO = 0x0A,
		HA_MAGNETO = 0x0B,
		HA_KEYPAD = 0x0C,
		HA_SCREEN = 0x0D,
		HA_SPEAKER = 0x0E,
		HA_LED = 0x10,


		HA_CAN = 0x66,


		API_POST = 0xA1,
		API_RAW_STEREO_CAMERA = 0xA2,
		API_GPS = 0xA3,
		API_SMS = 0xA4,
		API_GPRS = 0xA5,
		API_STATUS = 0xA6,
		API_COMMAND = 0xA7,
		API_MOTORS = 0xA8,
		API_MOVE_ACTUATOR = 0xA9,
		API_LIDAR = 0xAA,
		API_IHM_DISPLAY = 0xAB,
		API_IHM_ASK_ENUM = 0xAC,
		API_IHM_ASK_VALUE = 0xAD,
		API_RUN_PLOT_VALUE = 0xAE,
		API_ENUM_RESPONSE = 0xAF,
		API_VALUE_RESPONSE = 0xB0,
		API_PRESSED_IHM_BUTTON = 0xB1,
		API_MESSAGE = 0xB2,
		API_LOG_TO_ROBOT = 0xB3,
		API_WATCHDOG = 0xB4,

		API_AUTO_STATUS = 0xB5,

		API_CAMERA_INTRINSICS = 0xB6,
		API_CAMERA_EXTRINSICS = 0xB7
	};

	public:


	Naio01Codec();
	~Naio01Codec();

	bool decode( uint8_t *buffer, uint bufferSize, bool &packetHeaderDetected );

	BaseNaio01PacketPtr decodeOneWholePacket( uint8_t *buffer, uint bufferSize );

	bool firstPacketIdxAndSize( uint8_t *buffer, uint bufferSize, uint &firstPacketIdx, uint &firstPacketSize );

	uint8_t workingBuffer[2200000];

	std::vector<BaseNaio01PacketPtr> currentBasePacketList;

	void reset();

	private:

	uint maxCapacity = 2200000;
	int currentBufferPos = 0;
	uint currentMaxPacketSize = 0;
	uint currentPayloadSize = 0;
};


#endif //OZCORE_NAIO01CODEC_HPP
