//=================================================================================================
//
//  Copyright(c)  2013  Jean Inderchit
//
//  Vitals is free software: you can redistribute it and/or modify it under the terms of the GNU
//	General Public License as published by the Free Software Foundation, either version 3 of the
//	License, or (at your option) any later version.
//
//  Vitals is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//	even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with Vitals. If not,
//	see <http://www.gnu.org/licenses/>.
//
//=================================================================================================


//=================================================================================================
// I N C L U D E   F I L E S

#include "../include/oz440_api/CLBuffer.hpp"
#include <cstring>

//=================================================================================================
// C O N S T A N T S   &   L O C A L   C O D E

//#define DEBUG_BUFFER


//=================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//-------------------------------------------------------------------------------------------------
//
cl::Buffer::Buffer( const size_t sz )
	: data_( nullptr )
	, size_( sz )
	, owner_( true )
{
	if( size_ > 0 )
	{
		// Allocate the buffer and reset the data
		data_ = new uint8_t[size_];
		clear();

		#ifdef DEBUG_BUFFER
		cl::PrintLine( "New empty buffer created with size ", size_ );
		#endif
	}
	else
	{
		#ifdef DEBUG_BUFFER
		cl::PrintLine( "New empty buffer created" );
		#endif
	}
}

//-------------------------------------------------------------------------------------------------
//
cl::Buffer::Buffer( uint8_t* d, const size_t sz, bool own, bool deepCopy )
	: data_( nullptr )
	, size_( sz )
	, owner_( deepCopy ? true : own )
{
	//CL_ASSERT( size_ && "Size should not be null" );

	if( deepCopy )
	{
		data_ = new uint8_t[size_];
		std::memcpy( data_, d, size_ );

		#ifdef DEBUG_BUFFER
		cl::PrintLine( "New buffer created with deep copy" );
		#endif
	}
	else
	{
		data_ = d;

		#ifdef DEBUG_BUFFER
		if( owner_ )
		{
			cl::PrintLine( "New buffer created with ownership" );
		}
		else
		{
			cl::PrintLine( "New buffer created without ownership" );
		}
		#endif
	}
}

//-------------------------------------------------------------------------------------------------
//
cl::Buffer::~Buffer()
{
	cleanup();
}

//=================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//-------------------------------------------------------------------------------------------------
//
void
cl::Buffer::clear_chunk( uint8_t* ptr, const size_t sz )
{
	std::memset( ptr, 0x0, sz );
}

//-------------------------------------------------------------------------------------------------
//
uint8_t*
cl::Buffer::data()
{
	return data_;
}

//-------------------------------------------------------------------------------------------------
//
const uint8_t*
cl::Buffer::data() const
{
	return data_;
}

//-------------------------------------------------------------------------------------------------
//
uint8_t
cl::Buffer::operator[]( const size_t index ) const
{
	return data_[index];
}

//-------------------------------------------------------------------------------------------------
//
uint8_t&
cl::Buffer::operator[]( const size_t index )
{
	return data_[index];
}

//-------------------------------------------------------------------------------------------------
//
uint8_t
cl::Buffer::at( const size_t index ) const
{
	if( index > size_ )
	{
		throw std::out_of_range( "out of range access" );
	}

	return data_[index];
}

//-------------------------------------------------------------------------------------------------
//
uint8_t&
cl::Buffer::at( const size_t index )
{
	if( index > size_ )
	{
		throw std::out_of_range( "out of range access" );
	}

	return data_[index];
}

//-------------------------------------------------------------------------------------------------
//
size_t
cl::Buffer::size() const
{
	return size_;
}

//-------------------------------------------------------------------------------------------------
//
bool
cl::Buffer::is_owner() const
{
	return owner_;
}

//-------------------------------------------------------------------------------------------------
//
void
cl::Buffer::release_ownership()
{
	owner_ = false;
}

//-------------------------------------------------------------------------------------------------
//
void
cl::Buffer::cleanup()
{
	if( owner_ )
	{
		#ifdef DEBUG_BUFFER
		cl::PrintLine( "owned buffer deleted" );
		#endif

		delete[] data_;
		data_ = nullptr;
	}
}

//-------------------------------------------------------------------------------------------------
//
void
cl::Buffer::clear()
{
	// Clear the entire buffer
	clear_chunk( data_, size_ );
}

//-------------------------------------------------------------------------------------------------
//
void
cl::Buffer::resize( const size_t sz, bool clearNewMemory )
{
	if( !owner_ || size_ == sz )
	{
		return;
	}

	// Keep a pointer to the old data and create the new buffer
	uint8_t* oldPtr = data_;
	uint8_t* newPtr = new uint8_t[sz];

	if( sz > size_ )	// grow the buffer
	{
		// Move the old data into the new buffer
		data_ = reinterpret_cast<uint8_t*>( std::memcpy(newPtr, oldPtr, size_) );

		if( clearNewMemory )
		{
			// Make sure the grown buffer data is set to zero
			clear_chunk( data_ + size_, sz - size_ );
		}
	}
	else if( sz < size_ )	// shrink the buffer
	{
		// Move the old data into the new buffer
		data_ = reinterpret_cast<uint8_t*>( std::memcpy(newPtr, oldPtr, sz) );
	}

	// Delete the old buffer
	delete[] oldPtr;

	// Reset the size
	size_ = sz;
}

//-------------------------------------------------------------------------------------------------
//
void
cl::Buffer::insert( uint8_t* ptr, const size_t sz, const size_t pos )
{
	// Make sure we own the buffer before doing anything with it
	if( !owner_ )
	{
		return;
	}

	// new size
	size_t newSize = size_ + sz;
	size_t oldSize = size_;
	resize( newSize );

	// append the buffer at the end
	if( pos == oldSize )
	{
		std::memcpy( data_ + pos, ptr, sz );
	}
	else
	{
		resize( 0, 1 );
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

void
cl::Buffer::insert( UniqueBufferPtr buffer, const size_t pos )
{
	insert(buffer->data(),buffer->size(),pos);
}

//-------------------------------------------------------------------------------------------------
//
void
cl::Buffer::append( uint8_t* ptr, const size_t sz )
{
	insert( ptr, sz, size() );
}

//-------------------------------------------------------------------------------------------------
//
void
cl::Buffer::append( cl::Buffer::UniqueBufferPtr buffer )
{
	insert( buffer->data(), buffer->size(), size() );
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//------------------------------------------ UNIT TESTS -------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
/*
#include "../../include/TestClinic/TCUnitTest.h"


//=================================================================================================
// U N I T   T E S T S   C O D E   S E C T I O N

TC_DEFINE_UNIT_TEST( CLBufferUT )
{
	// Create an empty buffer
	cl::Buffer buffer(0);

	TC_TEST_DIE( buffer.size() == 0 );
	TC_TEST_DIE( buffer.is_owner() );

	buffer.resize( 2, true );
	TC_TEST_DIE( buffer.size() == 2 );
	TC_TEST_DIE( buffer.at(0) == 0 );
	TC_TEST_DIE( buffer.at(1) == 0 );

	buffer[0] = 1;
	buffer[1] = 6;

	TC_TEST_DIE( buffer.at(0) == 1 );
	TC_TEST_DIE( buffer.at(1) == 6 );

	buffer.resize(1);
	TC_TEST_DIE( buffer.size() == 1 );
	TC_TEST_DIE( buffer.at(0) == 1 );

	buffer.resize(1);
	TC_TEST_DIE( buffer.size() == 1 );
	TC_TEST_DIE( buffer.at(0) == 1 );

	buffer.resize(0);
	TC_TEST_DIE( buffer.size() == 0 );

	uint8_t* tmp = new uint8_t[5];
	tmp[0] = 30;
	tmp[1] = 31;
	tmp[2] = 32;
	tmp[3] = 33;
	tmp[4] = 34;

	buffer.insert( tmp, 5, 0 );
	TC_TEST_DIE( buffer.size() == 5 );
	TC_TEST_DIE( buffer[0] == 30 );
	TC_TEST_DIE( buffer[1] == 31 );
	TC_TEST_DIE( buffer[2] == 32 );
	TC_TEST_DIE( buffer[3] == 33 );
	TC_TEST_DIE( buffer[4] == 34 );

	tmp = new uint8_t[1];
	tmp[0] = 11;
	buffer.insert( tmp, 1, 2 );
	TC_TEST_DIE( buffer[2] == 11 );

	tmp = new uint8_t[1];
	tmp[0] = 22;
	buffer.append( tmp, 1 );
	TC_TEST_DIE( buffer[buffer.size()-1] == 22 );

	buffer.cleanup();

	return true;
}
TC_END_UNIT_TEST( CLBufferUT )
*/
