//=============================================================================
//
//  Copyright (C)  2013  Jean Inderchit
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//=============================================================================


//=============================================================================
// I N C L U D E   F I L E S

#include "vitals/CLByteConversion.h"


//=============================================================================
// C O N S T A N T S   &   L O C A L   C O D E

//=============================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N



//=============================================================================
// M E T H O D S   C O D E   S E C T I O N

// int16 to ubyte conversions (little endian)
cl::u8Array< 2 >
cl::i16_to_u8Array( const int16_t value )
{
	return detail::type_to_uint8< int16_t >( value );
}

// ubytes to int16 conversions (little endian)
int16_t
cl::u8Array_to_i16( const cl::u8Array< 2 >& arr )
{
	return detail::uint8_to_type< int16_t >( arr );
}

// uint16 to ubytes conversions (little endian)
cl::u8Array< 2 >
cl::u16_to_u8Array( const uint16_t value )
{
	return detail::type_to_uint8< uint16_t >( value );
}

// ubytes to uint16 conversions (little endian)
uint16_t
cl::u8Array_to_u16( const cl::u8Array< 2 >& arr )
{
	return detail::uint8_to_type< uint16_t >( arr );
}

// int32 to ubytes conversions (little endian)
cl::u8Array< 4 >
cl::i32_to_u8Array( const int32_t value )
{
	return detail::type_to_uint8< int32_t >( value );
}

// ubytes to int32 conversions (little endian)
int32_t
cl::u8Array_to_i32( const cl::u8Array< 4 >& arr )
{
	return detail::uint8_to_type< int32_t >( arr );
}

// uint32 to ubytes conversions (little endian)
cl::u8Array< 4 >
cl::u32_to_u8Array( const uint32_t value )
{
	return detail::type_to_uint8< uint32_t >( value );
}

// ubytes to uint32 conversions (little endian)
uint32_t
cl::u8Array_to_u32( const cl::u8Array< 4 >& arr )
{
	return detail::uint8_to_type< uint32_t >( arr );
}

// int64 to ubytes conversions (little endian)
cl::u8Array< 8 >
cl::i64_to_u8Array( const int64_t value )
{
	return detail::type_to_uint8< int64_t >( value );
}

// ubytes to int64 conversions (little endian)
int64_t
cl::u8Array_to_i64( const cl::u8Array< 8 >& arr )
{
	return detail::uint8_to_type< int64_t >( arr );
}

// uint64 to ubytes conversions (little endian)
cl::u8Array< 8 >
cl::u64_to_u8Array( const uint64_t value )
{
	return detail::type_to_uint8< uint64_t >( value );
}

// ubytes to uint64 conversions (little endian)
uint64_t
cl::u8Array_to_u64( const cl::u8Array< 8 >& arr )
{
	return detail::uint8_to_type< uint64_t >( arr );
}

// float to uint8 conversions (little endian)
std::array< uint8_t, 4 >
cl::float_to_u8Array( const float value )
{
	static_assert( sizeof( float ) == 4, "sizeof(float) != 4" );

	std::array< uint8_t, 4 > bytes{ { } };
	const uint8_t* p = reinterpret_cast<const uint8_t*>( &value );

	for( size_t i = 0; i < 4; ++i )
	{
		bytes[i] = p[i];
	}

	return bytes;
}

// uint8 to float conversions (little endian)
float
cl::u8Array_to_float( const std::array< uint8_t, 4 >& arr )
{
	static_assert( sizeof( float ) == 4, "sizeof(float) != 4" );
	float result{ };

	std::copy( reinterpret_cast<const int8_t*>(&arr[0]),
			   reinterpret_cast<const int8_t*>(&arr[4]),
			   reinterpret_cast<int8_t*>(&result) );

	return result;
}

// double to uint8 conversions (little endian)
std::array< uint8_t, 8 >
cl::double_to_u8Array( const double value )
{
	static_assert( sizeof( double ) == 8, "sizeof(double) != 8" );

	std::array< uint8_t, 8 > bytes{ { } };
	const uint8_t* p = reinterpret_cast<const uint8_t*>( &value );

	for( size_t i = 0; i < 8; ++i )
	{
		bytes[i] = p[i];
	}

	return bytes;
}

// uint8 to double conversions (little endian)
double
cl::u8Array_to_double( const std::array< uint8_t, 8 >& arr )
{
	static_assert( sizeof( double ) == 8, "sizeof(double) != 8" );
	double result{ };

	std::copy( reinterpret_cast<const int8_t*>(&arr[0]),
			   reinterpret_cast<const int8_t*>(&arr[8]),
			   reinterpret_cast<int8_t*>(&result) );

	return result;
}
