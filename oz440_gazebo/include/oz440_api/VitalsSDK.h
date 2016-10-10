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

#ifndef VITALSSDK_H
#define VITALSSDK_H

//=================================================================================================
// I N C L U D E   F I L E S   A N D   F O R W A R D   D E C L A R A T I O N S

//=================================================================================================
// C O N S T A N T S / M A C R O S

/// C++ visibility support.
/// See http://gcc.gnu.org/wiki/Visibility for more information

#ifdef DECL_SPEC
#define VITALS_SDK      __attribute__((visibility("default")))
#define VITALS_LOCAL    __attribute__((visibility ("hidden")))
#else
#define VITALS_SDK
#define VITALS_LOCAL
#endif


//=================================================================================================
// C L A S S E S

//=================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif
