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

#ifndef CLPIMPL_HPP
#define CLPIMPL_HPP

//==================================================================================================
// I N C L U D E   F I L E S

#include <memory>

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

namespace cl
{

template< typename T >
class pimpl
{
//--Methods-----------------------------------------------------------------------------------------
public:
	pimpl();

	template< typename ...Args >
	pimpl( Args&& ... );

	pimpl( const T& other );

	pimpl( pimpl&& other );

	~pimpl();

	T* operator->() const;

	T& operator*() const;

	void swap( pimpl& other );

//--Data members------------------------------------------------------------------------------------
private:
	std::unique_ptr<T> m_;
};

} // cl

//==================================================================================================
// T E M P L A T E S   C O D E   S E C T I O N

//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif  // CLPIMPL_HPP
