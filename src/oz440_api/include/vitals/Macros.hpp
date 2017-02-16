//==================================================================================================
//
//  Copyright(c) 2006, 2009, 2011 Martin Girard, Mathieu Larose, 2012, 2013 Jean Inderchit
//
//  Vitals is free software: you can redistribute it and/or modify it under the terms of the GNU
//  General Public License as published by the Free Software Foundation, either version 3 of the
//  License, or (at your option) any later version.
//
//  Vitals is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//  even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with Vitals.
// If not, see <http://www.gnu.org/licenses/>.
//
//==================================================================================================

#ifndef COPY_MACRO_HPP
#define COPY_MACRO_HPP

//==================================================================================================
// I N C L U D E   F I L E S

#include <cstring>

//==================================================================================================
// C O N S T A N T S / M A C R O S

#define STR_HELPER( x ) #x
#define STR( x ) STR_HELPER(x)

#if defined(__GNUC__)
#if defined(__GNUC_PATCHLEVEL__)
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#else
		#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100)
	#endif
#endif

/// Defines the common error code returned by many C/Linux functions when they fail.
#define C_API_ERR		(-1)
#define C_API_OK		0
#define CL_CLEAR(x)		memset( &(x), 0, sizeof (x) )

/// Defines the origin point where it is placed.
#define CL_ORIGIN			__LINE__, __func__, __FILE__

/// Defines a generic macro to help generating enum names to values.
#define CL_CHECK_ENUM( rs, value )  if( rs == #value ) return value;

/// Counterpart macro used to "convert" actual enum values to their string.
#define CL_CHECK_ENUM_STR( rs, value )  if( rs == value ) return #value;


//==================================================================================================
// G L O B A L S

#endif // MACRO_HPP
