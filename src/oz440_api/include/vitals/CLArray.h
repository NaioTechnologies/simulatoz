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

#ifndef CLARRAY_H
#define CLARRAY_H

//=============================================================================
// I N C L U D E   F I L E S

#include <array>


//=============================================================================
// F O R W A R D   D E C L A R A T I O N S

//=============================================================================
// C O N S T A N T S

//=============================================================================
// C L A S S E S

namespace cl
{

#ifndef i8Array
template <size_t S>
using i8Array = std::array<int8_t, S>;
#endif

#ifndef u8Array
template <size_t S>
using u8Array = std::array<uint8_t, S>;
#endif

#ifndef i16Array
template <size_t S>
using i16Array = std::array<int16_t, S>;
#endif

#ifndef u16Array
template <size_t S>
using u16Array = std::array<uint16_t, S>;
#endif

#ifndef i32Array
template <size_t S>
using i32Array = std::array<int32_t, S>;
#endif

#ifndef u32Array
template <size_t S>
using u32Array = std::array<uint32_t, S>;
#endif

#ifndef i64Array
template <size_t S>
using i64Array = std::array<int64_t, S>;
#endif

#ifndef u64Array
template <size_t S>
using u64Array = std::array<uint64_t, S>;
#endif

#ifndef fArray
template <size_t S>
using fArray = std::array<float, S>;
#endif

#ifndef dArray
template <size_t S>
using dArray = std::array<double, S>;
#endif

} // cl

//=============================================================================
// T E M P L A T E S   C O D E   S E C T I O N

//=============================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif  // end of CLARRAY_H
