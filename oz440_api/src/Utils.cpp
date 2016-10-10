//==================================================================================================
//
//  Copyright(c)  2013 - 2015  Jean Inderchit
//
//  This program is free software: you can redistribute it and/or modify it under the terms of the
//  GNU General Public License as published by the Free Software Foundation, either version 3 of
//  the License, or (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
//  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//  See the GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License along with This program.
//  If not, see <http://www.gnu.org/licenses/>.
//
//==================================================================================================

//==================================================================================================
// I N C L U D E   F I L E S

#include "../include/oz440_api/Utils.hpp"

#include <iostream>

//==================================================================================================
// C O N S T A N T S   &   L O C A L   C O D E

//--------------------------------------------------------------------------------------------------
//
void
util::terminate( const size_t line, const char* func, const char* file )
{
	std::cout << "terminated from \'" << func << "()\' in " << file << ", at line #" << line;
	std::terminate();
}


//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//------------------------------------------- UNIT TESTS -------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

#ifdef DEBUG

#include "TCUnitTest.h"


//==================================================================================================
// U N I T   T E S T S   C O D E   S E C T I O N

#endif
