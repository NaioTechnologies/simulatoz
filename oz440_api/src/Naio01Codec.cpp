#include <vitals/CLArray.h>
#include <vitals/CLByteConversion.h>
#include "Naio01Codec.hpp"
#include "ApiPostPacket.hpp"
#include "ApiGpsPacket.hpp"
#include "ApiSmsPacket.hpp"
#include "ApiGprsPacket.hpp"
#include "ApiStatusPacket.hpp"
#include "ApiCommandPacket.hpp"
#include "ApiMotorsPacket.hpp"
#include "ApiMoveActuatorPacket.hpp"
#include "ApiLidarPacket.hpp"
#include "ApiIhmDisplayPacket.hpp"
#include "ApiIhmAskEnumPacket.hpp"
#include "ApiIhmAskValuePacket.hpp"
#include "ApiRunPlotPacket.hpp"
#include "HaAcceleroPacket.hpp"
#include "HaLidarPacket.hpp"
#include "HaActuatorPacket.hpp"
#include "HaGpsPacket.hpp"
#include "HaGyroPacket.hpp"
#include "HaMagnetoPacket.hpp"
#include "HaMotorsPacket.hpp"
#include "HaOdoPacket.hpp"
#include "HaDS4RemotePacket.hpp"
#include "HaScreenPacket.hpp"
#include "HaSpeakerPacket.hpp"
#include "HaKeypadPacket.hpp"
#include "HaLedPacket.hpp"
#include "ApiEnumResponsePacket.hpp"
#include "ApiValueResponsePacket.hpp"
#include "HaCanPacket.hpp"
#include "ApiAutoStatusPacket.hpp"
#include "ApiPressedIhmButtonPacket.hpp"
#include "ApiMessagePacket.hpp"
#include "ApiLogToRobotPacket.hpp"
#include "ApiWatchdogPacket.hpp"
#include "ApiStereoCameraPacket.hpp"

//=============================================================================
//
Naio01Codec::Naio01Codec() :
		currentBasePacketList{ },
		maxCapacity{ 1000000 },
		currentBufferPos{0},
		currentMaxPacketSize{ 5000000 },
		currentPayloadSize{ 0 }
{

}

//=============================================================================
//
Naio01Codec::~Naio01Codec()
{

}

//=============================================================================
//
void Naio01Codec::reset()
{
	currentBufferPos = 0;
}

//=============================================================================
//
BaseNaio01PacketPtr Naio01Codec::decodeOneWholePacket( uint8_t *buffer, uint bufferSize )
{
	BaseNaio01PacketPtr packet = nullptr;

	if( bufferSize > 6 )
	{
		Naio01CodecPacketType packetType = static_cast<Naio01CodecPacketType>( buffer[ 6 ] );

		if( bufferSize >= 10 )
		{
			cl::u8Array< 4 > payloadSizeBuffer;

			payloadSizeBuffer[0] = buffer[ 7 ];
			payloadSizeBuffer[1] = buffer[ 8 ];
			payloadSizeBuffer[2] = buffer[ 9 ];
			payloadSizeBuffer[3] = buffer[ 10 ];

			uint32_t payloadSize = cl::u8Array_to_u32( payloadSizeBuffer );
			uint32_t wholePacketSize = payloadSize + 6 + 1 + 4 + 4;

			if( bufferSize == wholePacketSize )
			{
				switch( packetType )
				{
					case Naio01CodecPacketType::HA_CAN:
						packet = std::make_shared<HaCanPacket>();
						break;
					case Naio01CodecPacketType::HA_LIDAR:
						packet = std::make_shared<HaLidarPacket>();
						break;
					case Naio01CodecPacketType::HA_ACCELERO:
						packet = std::make_shared<HaAcceleroPacket>();
						break;
					case Naio01CodecPacketType::HA_ACTUATOR:
						packet = std::make_shared<HaActuatorPacket>();
						break;
					case Naio01CodecPacketType::HA_GPS:
						packet = std::make_shared<HaGpsPacket>();
						break;
					case Naio01CodecPacketType::HA_GYRO:
						packet = std::make_shared<HaGyroPacket>();
						break;
					case Naio01CodecPacketType::HA_MAGNETO:
						packet = std::make_shared<HaMagnetoPacket>();
						break;
					case Naio01CodecPacketType::HA_MOTORS:
						packet = std::make_shared<HaMotorsPacket>();
						break;
					case Naio01CodecPacketType::HA_ODO:
						packet = std::make_shared<HaOdoPacket>();
						break;
					case Naio01CodecPacketType::HA_DS4REMOTE:
						packet = std::make_shared<HaDS4RemotePacket>();
						break;
					case Naio01CodecPacketType::HA_SCREEN:
						packet = std::make_shared<HaScreenPacket>();
						break;
					case Naio01CodecPacketType::HA_SPEAKER:
						packet = std::make_shared<HaSpeakerPacket>();
						break;
					case Naio01CodecPacketType::HA_KEYPAD:
						packet = std::make_shared<HaKeypadPacket>();
						break;
					case Naio01CodecPacketType::HA_LED:
						packet = std::make_shared<HaLedPacket>();
						break;

					case Naio01CodecPacketType::API_WATCHDOG:
						packet = std::make_shared<ApiWatchdogPacket>();
						break;
					case Naio01CodecPacketType::API_AUTO_STATUS:
						packet = std::make_shared<ApiAutoStatusPacket>();
						break;
					case Naio01CodecPacketType::API_PRESSED_IHM_BUTTON:
						packet = std::make_shared<ApiPressedIhmButtonPacket>();
						break;
					case Naio01CodecPacketType::API_MESSAGE:
						packet = std::make_shared<ApiMessagePacket>();
						break;
					case Naio01CodecPacketType::API_LOG_TO_ROBOT:
						packet = std::make_shared<ApiLogToRobotPacket>();
						break;
					case Naio01CodecPacketType::API_POST:
						packet = std::make_shared<ApiPostPacket>();
						break;
					case Naio01CodecPacketType::API_RAW_STEREO_CAMERA:
						packet = std::make_shared<ApiStereoCameraPacket>();
						break;
					case Naio01CodecPacketType::API_GPS:
						packet = std::make_shared<ApiGpsPacket>();
						break;
					case Naio01CodecPacketType::API_SMS:
						packet = std::make_shared<ApiSmsPacket>();
						break;
					case Naio01CodecPacketType::API_GPRS:
						packet = std::make_shared<ApiGprsPacket>();
						break;
					case Naio01CodecPacketType::API_STATUS:
						packet = std::make_shared<ApiStatusPacket>();
						break;
					case Naio01CodecPacketType::API_COMMAND:
						packet = std::make_shared<ApiCommandPacket>();
						break;
					case Naio01CodecPacketType::API_MOTORS:
						packet = std::make_shared<ApiMotorsPacket>();
						break;
					case Naio01CodecPacketType::API_MOVE_ACTUATOR:
						packet = std::make_shared<ApiMoveActuatorPacket>();
						break;
					case Naio01CodecPacketType::API_LIDAR:
						packet = std::make_shared<ApiLidarPacket>();
						break;
					case Naio01CodecPacketType::API_IHM_DISPLAY:
						packet = std::make_shared<ApiIhmDisplayPacket>();
						break;
					case Naio01CodecPacketType::API_IHM_ASK_ENUM:
						packet = std::make_shared<ApiIhmAskEnumPacket>();
						break;
					case Naio01CodecPacketType::API_IHM_ASK_VALUE:
						packet = std::make_shared<ApiIhmAskValuePacket>();
						break;
					case Naio01CodecPacketType::API_RUN_PLOT_VALUE:
						packet = std::make_shared<ApiRunPlotPacket>();
						break;
					case Naio01CodecPacketType::API_ENUM_RESPONSE:
						packet = std::make_shared<ApiEnumResponsePacket>();
						break;
					case Naio01CodecPacketType::API_VALUE_RESPONSE:
						packet = std::make_shared<ApiValueResponsePacket>();
						break;
				}

				if( packet != nullptr )
				{
					//packet->decode( std::move(  cl::unique_buffer( buffer, wholePacketSize, false ) ) );
					packet->decode( buffer, wholePacketSize );
				}
			}
		}
	}

	return packet;
}

//=============================================================================
//
bool Naio01Codec::firstPacketIdxAndSize( uint8_t *buffer, uint bufferSize, uint &firstPacketIdx, uint &firstPacketSize )
{
	bool atLeastOnePacketHeaderAndSizeFound = false;
	bool headerFound = false;

	firstPacketIdx = 0;
	firstPacketSize = 0;

	if( bufferSize >= 11  )
	{
		uint idx = 0;

		while( idx < bufferSize and headerFound == false )
		{
			// header can be fetch enterely
			if( bufferSize >= ( idx + 5 ) )
			{
				if ((buffer[idx + 0] == 0x4e) and
					(buffer[idx + 1] == 0x41) and
					(buffer[idx + 2] == 0x49) and
					(buffer[idx + 3] == 0x4f) and
					(buffer[idx + 4] == 0x30) and
					(buffer[idx + 5] == 0x31))
				{
					firstPacketIdx = idx;
					headerFound = true;
				}
			}

			idx++;
		}

		if( headerFound == true )
		{
			if( bufferSize >= ( firstPacketIdx + 10 ) )
			{
				cl::u8Array< 4 > payloadSizeBuffer;

				payloadSizeBuffer[0] = buffer[firstPacketIdx + 7];
				payloadSizeBuffer[1] = buffer[firstPacketIdx +8];
				payloadSizeBuffer[2] = buffer[firstPacketIdx +9];
				payloadSizeBuffer[3] = buffer[firstPacketIdx +10];

				uint32_t payloadSize = cl::u8Array_to_u32( payloadSizeBuffer );

				firstPacketSize = payloadSize + 6 + 1 + 4 + 4;

				atLeastOnePacketHeaderAndSizeFound = true;
			}
		}
	}

	return atLeastOnePacketHeaderAndSizeFound;
}

//=============================================================================
//
bool Naio01Codec::decode( uint8_t *buffer, uint bufferSize, bool &packetHeaderDetected )
{
	bool atLeastOnePacketDecoded = false;

	packetHeaderDetected = false;

	uint idx = 0;

	if( currentBufferPos < 0 )
	{
		currentBufferPos = 0;
	}

	while( idx < bufferSize )
	{
		workingBuffer[ static_cast<uint>( currentBufferPos ) ] = buffer[ idx ];

		if( currentBufferPos == 0 and workingBuffer[0] != 0x4e )
		{
			currentBufferPos = -1;
		}
		else if( currentBufferPos == 1 and workingBuffer[1] != 0x41 )
		{
			currentBufferPos = -1;
		}
		else if( currentBufferPos == 2 and workingBuffer[2] != 0x49 )
		{
			currentBufferPos = -1;
		}
		else if( currentBufferPos == 3 and workingBuffer[3] != 0x4f )
		{
			currentBufferPos = -1;
		}
		else if( currentBufferPos == 4 and workingBuffer[4] != 0x30 )
		{
			currentBufferPos = -1;
		}
		else if( currentBufferPos == 5 and workingBuffer[5] != 0x31 )
		{
			currentBufferPos = -1;
		}
		//TEST PROTOCOL VERSION
		else if( currentBufferPos == 5 )
		{
			// validate header NAIO01
			if ((workingBuffer[0] == 0x4e) and
				(workingBuffer[1] == 0x41) and
				(workingBuffer[2] == 0x49) and
				(workingBuffer[3] == 0x4f) and
				(workingBuffer[4] == 0x30) and
				(workingBuffer[5] == 0x31))
			{
				packetHeaderDetected = true;
			}
		}
		// REACH PROTOCOL PACKET ID
		else if( currentBufferPos == 6 )
		{

		}
		// PACKET SIZE REACHED
		else if( currentBufferPos == 10 )
		{
			cl::u8Array< 4 > payloadSizeBuffer;

			payloadSizeBuffer[0] = workingBuffer[7];
			payloadSizeBuffer[1] = workingBuffer[8];
			payloadSizeBuffer[2] = workingBuffer[9];
			payloadSizeBuffer[3] = workingBuffer[10];

			currentPayloadSize =  cl::u8Array_to_u32( payloadSizeBuffer );
		}
		// WHOLE PACKET RECEIVED TRY DECODING
		else if( currentBufferPos == static_cast<int>( 6 + 1 + 4 + currentPayloadSize + 4 - 1 ) )
		{
			BaseNaio01PacketPtr packet = decodeOneWholePacket( workingBuffer, ( 6 + 1 + 4 + currentPayloadSize + 4 ) );

			if( packet != nullptr )
			{
				currentBasePacketList.push_back( packet );

				atLeastOnePacketDecoded = true;
			}

			currentBufferPos = -1;
		}

		currentBufferPos++;

		idx++;
	}

	return atLeastOnePacketDecoded;
}