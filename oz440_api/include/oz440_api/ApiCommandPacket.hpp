#ifndef OZCORE_APICOMMANDPACKET_HPP
#define OZCORE_APICOMMANDPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiCommandPacket : public BaseNaio01Packet
{
public:
	enum CommandType : uint8_t
	{
		START_API_MODE = 0x01,
		STOP_API_MODE = 0x02,
		WORK_PAUSE = 0x03,
		WORK_RESUME = 0x04,
		WORK_STOP = 0x05
	};

public:
	ApiCommandPacket( );
	ApiCommandPacket( CommandType commandType_ );
	~ApiCommandPacket( );

	virtual cl::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_COMMAND );
	}

public:
	CommandType commandType;
};

typedef std::shared_ptr<ApiCommandPacket> ApiCommandPacketPtr;

#endif //OZCORE_APICOMMANDPACKET_HPP
