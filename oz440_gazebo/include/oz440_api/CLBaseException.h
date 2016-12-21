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

#ifndef COPY_CLBASEEXCEPTION_H
#define COPY_CLBASEEXCEPTION_H

//=================================================================================================
// I N C L U D E   F I L E S

#include <exception>

//=================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//=================================================================================================
// C O N S T A N T S

//=================================================================================================
// C L A S S E S

namespace cl
{

/// Base class for exceptions
///
class BaseException
	: public std::exception
{
//--Methods----------------------------------------------------------------------------------------
protected:
	BaseException();

public:
	virtual ~BaseException();

	/// std::exception overrides
	virtual const char* what() const noexcept = 0;

//--Data members-----------------------------------------------------------------------------------
private:
};

}

//=================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif
