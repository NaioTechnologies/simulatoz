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

#ifndef COPY_CLEXCEPTION_H
#define COPY_CLEXCEPTION_H

//=================================================================================================
// I N C L U D E   F I L E S   A N D   F O R W A R D   D E C L A R A T I O N S

#include "CLBaseException.h"

#include <system_error>

//=================================================================================================
// C O N S T A N T S

namespace cl
{

/// Macro that allows the creation of custom exceptions easily
#define MyException( EName, EMessage )                                                        \
    class EName                                                                        \
        : public cl::Exception                                                                    \
    {                                                                                            \
    public:                                                                                        \
        EName() : Exception( EMessage )                                                        \
        { }                                                                                        \
                                                                                                \
        explicit EName( const size_t line, const char* func, const char* file )                    \
            : Exception( EMessage, line, func, file )                                            \
        { }                                                                                        \
                                                                                                \
        explicit EName( const std::string& msg, const size_t line, const char* func,            \
                        const char* file )                                                        \
            : Exception( std::string(EMessage) + " " + msg, line, func, file )                \
        { }                                                                                        \
                                                                                                \
        EName( const EName& other ) : Exception( other.what() )                                \
        { }                                                                                        \
                                                                                                \
        virtual ~EName() { }                                                                    \
    }


//=================================================================================================
// C L A S S E S

/// Super class for all exception handle in the CL module class structure.
class SystemError
	: public BaseException
		, private std::system_error
{
//--Methods----------------------------------------------------------------------------------------
public:
	explicit SystemError( const int32_t ec,
						  const std::error_category& category = std::generic_category(),
						  const size_t line = 0, const char* func = "", const char* file = "" );

	SystemError( const SystemError& other );

	virtual ~SystemError();

	/// std::exception overrides
	virtual const char* what() const noexcept final;

	const std::error_code& error_code() const noexcept;

	/// Returns a string with the location where the system error was thrown.
	virtual std::string where() const noexcept final;

//--Data members-----------------------------------------------------------------------------------
private:
	std::string where_;
};

/// Super class for all exception handle in the CL module class structure.
class Exception
	: public BaseException
{
//--Methods----------------------------------------------------------------------------------------
public:
	explicit Exception( const std::string& message );

	/// Exception used for assertions.
	explicit Exception( const std::string& message, const size_t line, const char* file );

	/// Exception with localization on the thrown exception.
	explicit Exception( const std::string& message, const size_t line, const char* func,
						const char* file );

//	/// Exception with localization used for the low level C error.
//	explicit CLException( const std::string& message, const size_t line, const char* func,
//						  const char* file, const int errorCode );
//
//	explicit CLException( const int errc, const std::error_category& category, const size_t line,
//						  const char* func, const char* file );

	Exception( const Exception& other );

	virtual ~Exception();

	/// std::exception overrides
	const char* what() const noexcept final;

//--Data members-----------------------------------------------------------------------------------
private:
	std::string message_;
};

}

//=================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif
