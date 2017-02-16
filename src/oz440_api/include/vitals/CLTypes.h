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

#ifndef CLTYPES_H
#define CLTYPES_H

//=================================================================================================
// I N C L U D E   F I L E S

#include <cstddef>
#include <cstdint>


//=================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//=================================================================================================
// C O N S T A N T S

/// These types are set for the folowing platforms:
///     - ILP32 or 4/4/4 (int, long, and pointer are 32-bit)
///     - LLP64 or 4/4/8 (int and long are 32-bit, pointer is 64-bit)
///     - LP64 or 4/8/8 (int is 32-bit, long and pointer are 64-bit)

#ifndef ubyte
typedef uint8_t ubyte;    // Is always 8 bits
#endif

#ifndef sbyte
typedef int8_t sbyte;    // Is always 8 bits
#endif

#ifndef uchar
typedef unsigned char uchar;    // Always 8 bits
#endif

#ifndef schar
typedef signed char schar;    // Always 8 bits
#endif

#ifndef ushort
typedef unsigned short ushort;  // At least 16 bits
#endif

#ifndef uint
typedef unsigned int uint;      // Always 32 bits
#endif

#ifndef ulong
typedef unsigned long ulong;    // 32 bits on Unix and Win32 API, 64 bits on Win64 API
#endif

#ifndef longlong
typedef long long longlong;     // Always 64 bits
#endif

#ifndef ulonglong
typedef unsigned long long ulonglong;   // Always 64 bits
#endif


// Fixed width integer types

#ifndef uint8
typedef uint8_t		uint8;	// 8 bits unsigned integer
#endif

#ifndef int8
typedef int8_t		int8;	// 8 bits signed integer
#endif

#ifndef uint16
typedef uint16_t	uint16;	// 16 bits unsigned integer
#endif

#ifndef int16
typedef int16_t		int16;	// 16 bits signed integer
#endif

#ifndef uint32
typedef uint32_t	uint32;	// 32 bits unsigned integer
#endif

#ifndef int32
typedef int32_t		int32;	// 32 bits signed integer
#endif

#ifndef uint64
typedef uint64_t	uint64;	// 64 bits unsigned integer
#endif

#ifndef int64
typedef int64_t		int64;	// 64 bits signed integer
#endif


//=================================================================================================
// C L A S S E S

//=================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N


#endif
