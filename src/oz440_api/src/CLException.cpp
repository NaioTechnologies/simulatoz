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

#include "vitals/CLException.h"
#include "vitals/Format.hpp"

#include <cstring>


//=================================================================================================
// C O N S T A N T (S)   &   L O C A L   C O D E

//=================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//-------------------------------------------------------------------------------------------------
//
cl::Exception::Exception( const std::string& message )
	: BaseException()
	, message_( message )
{ }

//-------------------------------------------------------------------------------------------------
//
cl::Exception::Exception( const std::string& message, const size_t line, const char* file )
	: BaseException()
	, message_( message )
{
	// The is an assertion
	message_.insert( 0, "Assertion \'" );

	std::string origin( tfm::format( "\' failed ( in %s, at line #%i)", file, line ) );

	// Append the localization of the exception
	message_.append( origin );
}

//-------------------------------------------------------------------------------------------------
//
cl::Exception::Exception( const std::string& message, const size_t line, const char* func,
						  const char* file )
	: BaseException()
	, message_( message )
{
	std::string origin( tfm::format( " (%s: %s, at line #%i)", func, file, line ) );

	// Append the localization of the exception
	message_.append( origin );
}

//-------------------------------------------------------------------------------------------------
//
cl::Exception::Exception( const Exception& other )
	: message_( other.message_ )
{ }

//-------------------------------------------------------------------------------------------------
//
cl::Exception::~Exception()
{ }


//=================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//-------------------------------------------------------------------------------------------------
//
const char*
cl::Exception::what() const noexcept
{
	return message_.c_str();
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------- CLSystemException ----------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
//
cl::SystemError::SystemError( const int32_t errcode, const std::error_category& category,
							  const size_t line, const char* func, const char* file )
	: BaseException()
	, std::system_error( errcode, category )
{
	where_ = tfm::format( " (%s: %s, at line #%i)", func, file, line );
}

//-------------------------------------------------------------------------------------------------
//
cl::SystemError::SystemError( const SystemError& other )
	: BaseException()
	, std::system_error( other.error_code() )
	, where_( other.where_ )
{
	std::system_error( EFAULT, std::system_category() );
}

//-------------------------------------------------------------------------------------------------
//
cl::SystemError::~SystemError()
{ }

//-------------------------------------------------------------------------------------------------
//
const char*
cl::SystemError::what() const noexcept
{
	return std::system_error::what();
}

//-------------------------------------------------------------------------------------------------
//
const std::error_code&
cl::SystemError::error_code() const noexcept
{
	return code();
}

//-------------------------------------------------------------------------------------------------
//
std::string
cl::SystemError::where() const noexcept
{
	return where_;
}


//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//------------------------------------------ UNIT TESTS -------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------


//=================================================================================================
// U N I T   T E S T   C O D E   S E C T I O N
