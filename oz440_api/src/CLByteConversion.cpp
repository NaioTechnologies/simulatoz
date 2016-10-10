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

#include "../include/oz440_api/CLByteConversion.h"


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
cl::float32_to_u8Array( const float value )
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
cl::u8Array_to_float32( const std::array< uint8_t, 4 >& arr )
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
cl::float64_to_u8Array( const double value )
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
cl::u8Array_to_float64( const std::array< uint8_t, 8 >& arr )
{
	static_assert( sizeof( double ) == 8, "sizeof(double) != 8" );
	double result{ };

	std::copy( reinterpret_cast<const int8_t*>(&arr[0]),
			   reinterpret_cast<const int8_t*>(&arr[8]),
			   reinterpret_cast<int8_t*>(&result) );

	return result;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-------------------------------- UNIT TESTS ---------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
/*
#include "../../include/TestClinic/TCUnitTest.h"
#include "../include/CLMath.h"


//=============================================================================
// U N I T   T E S T S   C O D E   S E C T I O N

TC_DEFINE_UNIT_TEST( CLByteConversionUT )
	{
		using namespace cl;

		// Check integers
		static_assert( sizeof( short ) == 16 / 8, "sizeof(short) != 16/8" );
		static_assert( sizeof( ushort ) == 16 / 8, "sizeof(ushort) != 16/8" );
		static_assert( sizeof( int ) == 32 / 8, "sizeof(int) != 32/8" );
		static_assert( sizeof( uint ) == 32 / 8, "sizeof(uint) != 32/8" );
		//static_assert( sizeof(ulong)	 == 64/8, "sizeof(ulong) != 64/8" );
		static_assert( sizeof( ulonglong ) == 64 / 8, "sizeof(ulonglong) != 64/8" );

		// Check floating points
		static_assert( sizeof( float ) == 32 / 8, "sizeof(float) != 32/8" );
		static_assert( sizeof( double ) == 64 / 8, "sizeof(double) != 64/8" );

		//=========================================================================
		// int16 to uint8 conversion and uint8 to int16 conversion
		std::array< uint8_t, 2 > byteArr1{ { } };
		int16_t res1{ };

		byteArr1 = i16_to_u8Array( cl::math::min_limit< int16_t >() );
		res1 = u8Array_to_i16( byteArr1 );
		TC_TEST_DIE( res1 == cl::math::min_limit< int16_t >() );

		byteArr1 = i16_to_u8Array( static_cast<int16_t>(cl::math::max_limit< int16_t >() / 2) );
		res1 = u8Array_to_i16( byteArr1 );
		TC_TEST_DIE( res1 == cl::math::max_limit< int16_t >() / 2 );

		byteArr1 = i16_to_u8Array( cl::math::max_limit< int16_t >() );
		res1 = u8Array_to_i16( byteArr1 );
		TC_TEST_DIE( res1 == cl::math::max_limit< int16_t >() );

		//-------------------------------------------------------------------------
		// uint16 to ubyte conversion and ubyte to uint16 conversion
		std::array< uint8_t, 2 > byteArr2{ { } };
		uint16_t res2{ };

		byteArr2 = u16_to_u8Array( cl::math::min_limit< uint16_t >() );
		res2 = u8Array_to_u16( byteArr2 );
		TC_TEST_DIE( res2 == cl::math::min_limit< uint16_t >() );

		byteArr2 = u16_to_u8Array( static_cast<uint16_t>(cl::math::max_limit< uint16_t >() / 2) );
		res2 = u8Array_to_u16( byteArr2 );
		TC_TEST_DIE( res2 == cl::math::max_limit< uint16_t >() / 2 );

		byteArr2 = u16_to_u8Array( cl::math::max_limit< uint16_t >() );
		res2 = u8Array_to_u16( byteArr2 );
		TC_TEST_DIE( res2 == cl::math::max_limit< uint16_t >() );

		//=========================================================================
		// ubytes to int32_t conversions and int32_t to ubytes conversion
		std::array< uint8_t, 4 > byteArr3{ { } };
		int32_t res3{ };

		byteArr3 = i32_to_u8Array( cl::math::min_limit< int32_t >() );
		res3 = u8Array_to_i32( byteArr3 );
		TC_TEST_DIE( res3 == cl::math::min_limit< int32_t >() );

		byteArr3 = i32_to_u8Array( cl::math::max_limit< int32_t >() / 2 );
		res3 = u8Array_to_i32( byteArr3 );
		TC_TEST_DIE( res3 == cl::math::max_limit< int32_t >() / 2 );

		byteArr3 = i32_to_u8Array( cl::math::max_limit< int32_t >() );
		res3 = u8Array_to_i32( byteArr3 );
		TC_TEST_DIE( res3 == cl::math::max_limit< int32_t >() );

		//-------------------------------------------------------------------------
		// ubytes to int32_t conversions and int32_t to ubytes conversion
		std::array< uint8_t, 4 > byteArr4{ { } };
		uint32_t res4{ };

		byteArr4 = u32_to_u8Array( cl::math::min_limit< uint32_t >() );
		res4 = u8Array_to_u32( byteArr4 );
		TC_TEST_DIE( res4 == cl::math::min_limit< uint32_t >() );

		byteArr4 = u32_to_u8Array( cl::math::max_limit< uint32_t >() / 2 );
		res4 = u8Array_to_u32( byteArr4 );
		TC_TEST_DIE( res4 == cl::math::max_limit< uint32_t >() / 2 );

		byteArr4 = u32_to_u8Array( cl::math::max_limit< uint32_t >() );
		res4 = u8Array_to_u32( byteArr4 );
		TC_TEST_DIE( res4 == cl::math::max_limit< uint32_t >() );

		//=========================================================================
		// ubytes to int64_t conversions and uint64_t to ubytes conversion
		std::array< uint8_t, 8 > byteArr5{ { } };
		int64_t res5{ };

		byteArr5 = i64_to_u8Array( cl::math::min_limit< int64_t >() );
		res5 = u8Array_to_i64( byteArr5 );
		TC_TEST_DIE( res5 == cl::math::min_limit< int64_t >() );

		byteArr5 = i64_to_u8Array( cl::math::max_limit< int64_t >() / 2 );
		res5 = u8Array_to_i64( byteArr5 );
		TC_TEST_DIE( res5 == cl::math::max_limit< int64_t >() / 2 );

		byteArr5 = i64_to_u8Array( cl::math::max_limit< int64_t >() );
		res5 = u8Array_to_i64( byteArr5 );
		TC_TEST_DIE( res5 == cl::math::max_limit< int64_t >() );

		//-------------------------------------------------------------------------
		// ubytes to uint64_t conversions and uint64_t to ubytes conversion
		std::array< uint8_t, 8 > byteArr6{ { } };
		uint64_t res6{ };

		byteArr6 = u64_to_u8Array( cl::math::min_limit< uint64_t >() );
		res6 = u8Array_to_u64( byteArr6 );
		TC_TEST_DIE( res6 == cl::math::min_limit< uint64_t >() );

		byteArr6 = u64_to_u8Array( cl::math::max_limit< uint64_t >() / 2 );
		res6 = u8Array_to_u64( byteArr6 );
		TC_TEST_DIE( res6 == cl::math::max_limit< uint64_t >() / 2 );

		byteArr6 = u64_to_u8Array( cl::math::max_limit< uint64_t >() );
		res6 = u8Array_to_u64( byteArr6 );
		TC_TEST_DIE( res6 == cl::math::max_limit< uint64_t >() );

		//-------------------------------------------------------------------------
		// ubytes to float conversions and float to ubytes conversion
		std::array< uint8_t, 4 > byteArr7{ { } };
		float res7{ };

		byteArr7 = float32_to_u8Array( cl::math::min_limit< float >() );
		res7 = u8Array_to_float32( byteArr7 );
		TC_TEST_DIE( cl::math::is_equal( res7, cl::math::min_limit< float >() ) );

		byteArr7 = float32_to_u8Array( cl::math::max_limit< float >() / 2 );
		res7 = u8Array_to_float32( byteArr7 );
		TC_TEST_DIE( cl::math::is_equal( res7, cl::math::max_limit< float >() / 2 ) );

		byteArr7 = float32_to_u8Array( cl::math::max_limit< float >() );
		res7 = u8Array_to_float32( byteArr7 );
		TC_TEST_DIE( cl::math::is_equal( res7, cl::math::max_limit< float >() ) );

		//-------------------------------------------------------------------------
		// ubytes to double conversions and double to ubytes conversion
		std::array< uint8_t, 8 > byteArr8{ { } };
		double res8{ };

		byteArr8 = float64_to_u8Array( cl::math::min_limit< double >() );
		res8 = u8Array_to_float64( byteArr8 );
		TC_TEST_DIE( cl::math::is_equal( res8, cl::math::min_limit< double >() ) );

		byteArr8 = float64_to_u8Array( cl::math::max_limit< double >() / 2 );
		res8 = u8Array_to_float64( byteArr8 );
		TC_TEST_DIE( cl::math::is_equal( res8, cl::math::max_limit< double >() / 2 ) );

		byteArr8 = float64_to_u8Array( cl::math::max_limit< double >() );
		res8 = u8Array_to_float64( byteArr8 );
		TC_TEST_DIE( cl::math::is_equal( res8, cl::math::max_limit< double >() ) );

		return true;
	}
TC_END_UNIT_TEST( CLByteConversionUT )
*/
