//=============================================================================
//
//  Copyright(c) 2006, 2009, 2011 Martin Girard, Mathieu Larose,
//  2012, 2013 Jean Inderchit
//
//  Vitals is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  Vitals is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the detailied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with Vitals. If not, see <http://www.gnu.org/licenses/>.
//
//=============================================================================

#ifndef COPY_CLBYTECONVERSION_H
#define COPY_CLBYTECONVERSION_H

//=============================================================================
// I N C L U D E   F I L E S

#include "CLTypes.h"
#include "CLArray.h"

#include <bitset>


//=============================================================================
// F O R W A R D   D E C L A R A T I O N S

//=============================================================================
// C O N S T A N T S

//=============================================================================
// C L A S S E S

namespace cl
{
namespace detail
{
template< typename T >
T uint8_to_type( const std::array< uint8_t, sizeof( T ) >& arr )
{
	T tmp{ };

	for( size_t i = 0; i < sizeof( T ); ++i )
	{
		tmp = static_cast<T>( (tmp << 8) | arr[i] );
	}

	return tmp;
}

template< typename T >
std::array< uint8_t, sizeof( T ) > type_to_uint8( const T value )
{
	std::array< uint8_t, sizeof( T ) > tmp{ { } };

	for( size_t i = 0; i < tmp.size(); ++i )
	{
		tmp[tmp.size() - i - 1] = static_cast<uint8_t>(
			(value >> 8 * i) & 0xff );
	}

	return tmp;
}

} // namespace detail

// int16 to ubyte conversions (little endian)
cl::u8Array< 2 > i16_to_u8Array( const int16_t value );

// ubytes to int16 conversions (little endian)
int16_t u8Array_to_i16( const cl::u8Array< 2 >& arr );

// uint16 to ubytes conversions (little endian)
cl::u8Array< 2 > u16_to_u8Array( const uint16_t value );

// ubytes to uint16 conversions (little endian)
uint16_t u8Array_to_u16( const cl::u8Array< 2 >& arr );

// int32 to ubytes conversions (little endian)
cl::u8Array< 4 > i32_to_u8Array( const int32_t value );

// ubytes to int32 conversions (little endian)
int32_t u8Array_to_i32( const cl::u8Array< 4 >& arr );

// uint32 to ubytes conversions (little endian)
cl::u8Array< 4 > u32_to_u8Array( const uint32_t value );

// ubytes to uint32 conversions (little endian)
uint32_t u8Array_to_u32( const cl::u8Array< 4 >& arr );

// int64 to ubytes conversions (little endian)
cl::u8Array< 8 > i64_to_u8Array( const int64_t value );

// ubytes to int64 conversions (little endian)
int64_t u8Array_to_i64( const cl::u8Array< 8 >& arr );

// uint64 to ubytes conversions (little endian)
cl::u8Array< 8 > u64_to_u8Array( const uint64_t value );

// ubytes to uint64 conversions (little endian)
uint64_t u8Array_to_u64( const cl::u8Array< 8 >& arr );

// float to uint8 conversions (little endian)
cl::u8Array< 4 > float_to_u8Array( const float value );

// uint8 to float conversions (little endian)
float u8Array_to_float( const cl::u8Array< 4 >& arr );

// double to uint8 conversions (little endian)
cl::u8Array< 8 > double_to_u8Array( const double value );

// uint8 to double conversions (little endian)
double u8Array_to_double( const cl::u8Array< 8 >& arr );

/// bitset -> uint8
template< size_t N >
inline uint8_t bitset_to_uint8( const std::bitset< N >& bitset )
{
	static_assert( N <= 8, "N should be equal or less that 8" );
	uint64_t val = bitset.to_ulong();
	return static_cast<uint8_t>(val);
}

/// uint8 -> bitset
template< size_t N >
inline std::bitset< N > uint8_to_bitset( const uint8_t b )
{
	static_assert( N <= 8, "N should be equal or less that 8" );
	return std::bitset< N >( b );
}
}    // cl

//=============================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif
