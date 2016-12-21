#ifndef OZCORE_APIPOSTPACKET_HPP
#define OZCORE_APIPOSTPACKET_HPP

#include "BaseNaio01Packet.hpp"
#include "Naio01Codec.hpp"


class ApiPostPacket : public BaseNaio01Packet
{
public:
	enum PostType : uint8_t
	{
		RED = 0x00,
		YELLOW = 0x01,
	};

	enum SourceCamera : uint8_t
	{
		LEFT = 0x00,
		RIGHT = 0x01,
	};

	struct Post
	{
		SourceCamera sourceCamera;
		PostType postType;
		float x;
		float y;

		Post()
		{
		}

		Post( SourceCamera sourceCamera_, PostType postType_, float x_, float y_ ) : sourceCamera(
			sourceCamera_ ), postType( postType_ ), x( x_ ), y( y_ )
		{
		}

		virtual ~Post(){ }
	};

public:
	ApiPostPacket( );

	~ApiPostPacket( );

	virtual cl_copy::BufferUPtr encode() override;

	virtual void decode( uint8_t *buffer, uint32_t bufferSize ) override;

	virtual uint8_t getPacketId() override
	{
		return static_cast<uint8_t>( Naio01Codec::Naio01CodecPacketType::API_POST );
	}

public:
	std::vector<Post> postList;
};

typedef std::shared_ptr<ApiPostPacket> ApiPostPacketPtr;

#endif //OZCORE_APIPOSTPACKET_HPP
