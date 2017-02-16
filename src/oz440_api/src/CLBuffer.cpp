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


//==================================================================================================
// I N C L U D E   F I L E S

#include "vitals/CLBuffer.hpp"
#include "vitals/Macros.hpp"
#include "vitals/CLException.h"
#include "vitals/Print.hpp"

#include <cassert>

//==================================================================================================
// C O N S T A N T S   &   L O C A L   C O D E

//#define DEBUG_BUFFER

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
cl_copy::Buffer::Buffer( const size_t sz )
	: data_{ }
	, size_{ sz }
	, owner_{ true }
	, parentBuffer_{ }
{
	if( size_ > 0 )
	{
		// Allocate the buffer and reset the data
		data_ = new uint8_t[size_];
		clear();

		#ifdef DEBUG_BUFFER
		util::println_sp( "Creating empty buffer of size:", size_ );
		#endif
	}
	else
	{
		#ifdef DEBUG_BUFFER
		util::println_sp( "New empty buffer created" );
		#endif
	}
}

//--------------------------------------------------------------------------------------------------
//
cl_copy::Buffer::Buffer( uint8_t* d, const size_t sz, bool own, bool deepCopy )
	: data_{ }
	, size_{ sz }
	, owner_{ deepCopy ? true : own }
	, parentBuffer_{ }
{
	if( size_ == 0 )
	{
		throw cl::Exception( "buffer size cannot not be null", CL_ORIGIN );
	}

	if( deepCopy )
	{
		data_ = new uint8_t[size_];
		std::memcpy( data_, d, size_ );

		#ifdef DEBUG_BUFFER
		util::println_sp( "New buffer created with deep copy" );
		#endif
	}
	else
	{
		data_ = d;

		#ifdef DEBUG_BUFFER
		if( owner_ )
		{
			util::println_sp( "New buffer created with ownership" );
		}
		else
		{
			util::println_sp( "New buffer created without ownership" );
		}
		#endif
	}
}

//--------------------------------------------------------------------------------------------------
//
cl_copy::Buffer::Buffer( UniqueBufferPtr parentBuffer, size_t offset, size_t s )
	: data_{ }
	, size_{ }
	, owner_{ }
	, parentBuffer_{ std::move( parentBuffer ) }
{
	assert( parentBuffer_ );

	if( offset + s > parentBuffer_->size() )
	{
		throw cl::Exception( "size is too big for the creation of sub-buffer" );
	}

	if( s == 0 )
	{
		s = parentBuffer_->size() - offset;
	}

	set_data_and_size( parentBuffer_->data() + offset, s );
}

//--------------------------------------------------------------------------------------------------
//
cl_copy::Buffer::~Buffer()
{
	cleanup();
}

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::clear_chunk( uint8_t* ptr, const size_t sz )
{
	std::memset( ptr, 0x00, sz );
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::set_data_and_size( uint8_t* d, const size_t s )
{
	data_ = d;
	size_ = s;
}

//--------------------------------------------------------------------------------------------------
//
uint8_t*
cl_copy::Buffer::data()
{
	return data_;
}

//--------------------------------------------------------------------------------------------------
//
const uint8_t*
cl_copy::Buffer::data() const
{
	return data_;
}

//--------------------------------------------------------------------------------------------------
//
uint8_t
cl_copy::Buffer::operator[]( const size_t index ) const
{
	return data_[index];
}

//--------------------------------------------------------------------------------------------------
//
uint8_t&
cl_copy::Buffer::operator[]( const size_t index )
{
	return data_[index];
}

//--------------------------------------------------------------------------------------------------
//
uint8_t
cl_copy::Buffer::at( const size_t index ) const
{
	if( index > size_ )
	{
		throw std::out_of_range( "cl::Buffer out of range access" );
	}

	return data_[index];
}

//--------------------------------------------------------------------------------------------------
//
uint8_t&
cl_copy::Buffer::at( const size_t index )
{
	if( index > size_ )
	{
		throw std::out_of_range( "cl::Buffer out of range access" );
	}

	return data_[index];
}

//--------------------------------------------------------------------------------------------------
//
size_t
cl_copy::Buffer::size() const
{
	return size_;
}

//--------------------------------------------------------------------------------------------------
//
bool
cl_copy::Buffer::is_owner() const
{
	return owner_;
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::release_ownership()
{
	owner_ = false;
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::cleanup()
{
	if( owner_ )
	{
		#ifdef DEBUG_BUFFER
		util::println_sp( "Deleting memory owned from buffer of size:", size_ );
		#endif

		delete[] data_;
		data_ = nullptr;
	}
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::clear()
{
	// Clear the entire buffer
	clear_chunk( data_, size_ );
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::resize( const size_t sz, bool clearNewMemory )
{
	if( !owner_ || size_ == sz )
	{
		return;
	}

	// Keep a pointer to the old data and create the new buffer
	uint8_t* oldPtr = data_;
	uint8_t* newPtr = new uint8_t[sz];

	if( sz > size_ )    // grow the buffer
	{
		if( size_ == 0 )
		{
			data_ = newPtr;
		}
		else
		{
			// Move the old data into the new buffer
			data_ = reinterpret_cast<uint8_t*>( std::memcpy( newPtr, oldPtr, size_ ) );
		}

		if( clearNewMemory )
		{
			// Make sure the grown buffer data is set to zero
			clear_chunk( data_ + size_, sz - size_ );
		}
	}
	else if( sz < size_ )    // shrink the buffer
	{
		// Move the old data into the new buffer
		data_ = reinterpret_cast<uint8_t*>( std::memcpy( newPtr, oldPtr, sz ) );
	}

	// Delete the old buffer
	delete[] oldPtr;

	// Reset the size
	size_ = sz;
}

//--------------------------------------------------------------------------------------------------
//
cl_copy::BufferUPtr
cl_copy::Buffer::takeout( const size_t sz, const size_t offset )
{
	// Make sure we own the buffer before doing anything with it.
	if( !owner_ || offset + sz > size_ )
	{
		return nullptr;
	}

	uint8_t* chunkPtr = new uint8_t[sz];

	if( offset == 0 ) // the chunk we are taking out is at the beggining of the buffer
	{
		std::memmove( chunkPtr, data_, sz );

		// Keep a pointer to the old data and create the new buffer
		size_t newSize = size_ - sz;
		uint8_t* newPtr = new uint8_t[newSize];

		std::memmove( newPtr, data_ + sz, newSize );

		delete[] data_;
		data_ = newPtr;
		size_ = newSize;
	}
	else if( offset + sz == size_ ) // the chunk we are taking out is at the end of the buffer
	{
		std::memmove( chunkPtr, data_ + offset, sz );
		resize( size_ - sz );

	}
	else // the chunk we are taking out is in the middle of the buffer
	{
		std::memmove( chunkPtr, data_ + offset, sz );

		// Keep a pointer to the old data and create the new buffer
		size_t newSize = size_ - sz;
		uint8_t* newPtr = new uint8_t[newSize];

		std::memmove( newPtr, data_, offset );
		std::memmove( newPtr + offset, data_ + offset + sz, newSize - offset );

		delete[] data_;
		data_ = newPtr;
		size_ = newSize;
	}

	return std::move( unique_buffer( chunkPtr, sz, true ) );
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::insert( uint8_t* ptr, const size_t sz, const size_t pos )
{
	// Make sure we own the buffer before doing anything with it
	if( !owner_ )
	{
		return;
	}

	// new size
	const size_t newSize = size_ + sz;
	const size_t oldSize = size_;

	resize( newSize );

	if( pos >= oldSize )    // append the buffer at the end
	{
		std::memcpy( data_ + oldSize, ptr, sz );
	}
	else if( pos == 0 )     // insert the buffer at beginning
	{
		std::memmove( data_ + sz, data_, oldSize );
		std::memcpy( data_, ptr, sz );
	}
	else                    // insert the buffer in the middle
	{
		uint8_t* tmp = new uint8_t[size_];

		// Move the first chunk of the old data into the new buffer
		std::memmove( tmp, data_, pos );

		// Move the buffer to insert in the middle
		std::memmove( tmp + pos, ptr, sz );

		// Move the second chunk of the old buffer into the new buffer
		std::memmove( tmp + pos + sz, data_ + pos, oldSize - pos );

		// Delete the old buffer and reset it with the new one
		delete[] data_;
		data_ = tmp;
	}
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::insert( BufferUPtr buffer, const size_t pos )
{
	Buffer* tmp = buffer.release();
	insert( tmp->data(), tmp->size(), pos );
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::append( uint8_t* ptr, const size_t sz )
{
	insert( ptr, sz, size() );
}

//--------------------------------------------------------------------------------------------------
//
void
cl_copy::Buffer::append( BufferUPtr buffer )
{
	if( !parentBuffer_ )
	{
		Buffer* tmp = buffer.release();
		insert( tmp->data(), tmp->size(), size() );
	}
}

//--------------------------------------------------------------------------------------------------
//
cl_copy::BufferUPtr
cl_copy::Buffer::clone()
{
	return std::move( unique_buffer( data_, size_, true, true ) );
}

//--------------------------------------------------------------------------------------------------
//
size_t
cl_copy::Buffer::write( const uint8_t* d, size_t s, size_t offset )
{
	if( !owner_ )
	{
		return 0;
	}

	size_t newSize = ((s + offset) > size_) ? s + offset : size_;

	// grow the buffer if it's too small
	if( newSize > size_ )
	{
		resize( newSize * 2 );
	}

	std::memcpy( data_ + offset, d, s );
	size_ = newSize;

	return s;
}

//--------------------------------------------------------------------------------------------------
//
size_t
cl_copy::Buffer::write( UniqueBufferPtr buffer, size_t offset )
{
	UniqueBufferPtr bufferPtr = std::move( buffer );
	return write( bufferPtr->data(), bufferPtr->size(), offset );
}

//--------------------------------------------------------------------------------------------------
//
size_t
cl_copy::Buffer::read( uint8_t* d, size_t s, size_t offset ) const
{
	if( (offset + s) > size_ )
	{
		return 0;
	}

	memcpy( d, data_ + offset, s );
	return s;
}


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//------------------------------------------- UNIT TESTS -------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//
//#ifdef DEBUG
//
//#include "TCUnitTest.h"
//
//
////==================================================================================================
//// U N I T   T E S T S   C O D E   S E C T I O N
//
//TC_DEFINE_UNIT_TEST( CLBufferUT )
//	{
//
//
//		return true;
//	}
//TC_END_UNIT_TEST( CLBufferUT )
//
//#endif
