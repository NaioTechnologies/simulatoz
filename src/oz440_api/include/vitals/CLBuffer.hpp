//==================================================================================================
//
//  Copyright(c)  2013 - 2015  Jean Inderchit
//
//  This program is free software: you can redistribute it and/or modify it under the terms of the
//  GNU General Public License as published by the Free Software Foundation, either version 3 of
//  the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with This program.
//  If not, see <http://www.gnu.org/licenses/>.
//
//==================================================================================================

#ifndef COPY_CLBUFFER_HPP
#define COPY_CLBUFFER_HPP

//==================================================================================================
// I N C L U D E   F I L E S

#include "Utils.hpp"

#include <memory>


//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

namespace cl_copy
{

/// This class encapsulates uint8_t buffers and takes care of the destruction of of the internal
/// data. The smart pointers should be used for sager memory management.
///
class Buffer
	: private util_copy::NonCopyable< Buffer >
{
	typedef std::shared_ptr< Buffer > BufferPtr;
	typedef std::unique_ptr< Buffer > UniqueBufferPtr;

	typedef const std::shared_ptr< Buffer > BufferConstPtr;

public:
	typedef std::allocator< uint8_t > allocator_type;


//--Methods-----------------------------------------------------------------------------------------
private:
	void clear_chunk( uint8_t* ptr, const size_t size );

public:
	explicit Buffer( const size_t size = 0 );

	explicit Buffer( uint8_t* data, const size_t size, bool own, bool deepCopy = false );

	/// Constructs a sub-buffer of the parent buffer. No allocation is performed
	/// nor deletion. Offset and size must fit in parent buffer size. We assume
	/// parent buffer is deleted after this sub-buffer. Caller must manage so.
	explicit Buffer( UniqueBufferPtr parentBuffer, size_t offset, size_t size );

	~Buffer();

	void set_data_and_size( uint8_t* data, const size_t size );

	uint8_t* data();

	const uint8_t* data() const;

	/// Return the byte at a specific index without boundaries checking.
	uint8_t operator[]( const size_t index ) const;

	uint8_t& operator[]( const size_t index );

	/// Return the byte at a specific index with boundaries checking
	uint8_t at( const size_t index ) const;

	uint8_t& at( const size_t index );

	void cleanup();

	size_t size() const;

	bool is_owner() const;

	void release_ownership();

	void clear();

	void resize( const size_t size, bool clearNewMemory = false );

	UniqueBufferPtr takeout( const size_t size, const size_t offset );

	void insert( uint8_t* ptr, const size_t size, const size_t pos );

	void insert( UniqueBufferPtr buffer, const size_t pos );

	void append( uint8_t* ptr, const size_t size );

	void append( UniqueBufferPtr buffer );

	UniqueBufferPtr clone();

	/// Write into the data buffer from a data pointer. Returns the number of byte written.
	/// The buffer will be resized automatically if needed.
	size_t write( const uint8_t* data, size_t size, size_t offset );

	/// Write into the data buffer from a std::unique_ptr. Returns the number of byte written.
	/// The buffer will be resized automatically if needed.
	size_t write( UniqueBufferPtr buffer, size_t offset );

	/// Copy the buffer from the offset position to a data pointer. Returns the number of bytes
	/// read from buffer. If offset+size is larger than the actual size of the buffer, fails and
	/// return 0.
	size_t read( uint8_t* data, size_t size, size_t offset ) const;

//--Data members------------------------------------------------------------------------------------
private:
	uint8_t* data_;
	size_t size_;
	bool owner_;

	BufferConstPtr parentBuffer_;
};

using BufferSPtr = std::shared_ptr< Buffer >;
using ConstBufferSPtr = std::shared_ptr< const Buffer >;

using BufferUPtr = std::unique_ptr< Buffer >;
using ConstBufferUPtr = std::unique_ptr< const Buffer >;

template< class... Args >
BufferSPtr shared_buffer( Args&& ... args )
{
	return std::make_shared< Buffer >( std::forward< Args >( args )... );
}

template< class... Args >
BufferUPtr unique_buffer( Args&& ... args )
{
	return std::make_unique< Buffer >( std::forward< Args >( args )... );
}

} // cl

//==================================================================================================
// T E M P L A T E S   C O D E   S E C T I O N

//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif  // CLBUFFER_HPP
