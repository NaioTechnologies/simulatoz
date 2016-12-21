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

#ifndef COPY_UTILS_HPP
#define COPY_UTILS_HPP

//==================================================================================================
// I N C L U D E   F I L E S   A N D   F O R W A R D   D E C L A R A T I O N S

#include <iosfwd>
#include <algorithm>

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// G L O B A L S

namespace util_copy
{

/// Utility function to turn off the unused parameter warning.
template< class... Args >
void ignore( Args&& ... )
{ }

/// Utility class to make the derived class non copyable
class noncopyable
{
public:
	noncopyable() = default;

	noncopyable( const noncopyable& ) = delete;

	noncopyable& operator=( const noncopyable& ) = delete;
};

template< class T >
class NonCopyable
{
protected:
	NonCopyable() = default;

public:
	NonCopyable( const NonCopyable& ) = delete;

	NonCopyable& operator=( const NonCopyable& ) = delete;
};

class nonmovable
{
public:
	nonmovable() = default;

	nonmovable( nonmovable&& ) = delete;

	nonmovable& operator=( nonmovable&& ) = delete;
};


[[noreturn]] void terminate( const size_t line, const char* func, const char* file );

template< typename Container, typename T >
auto
remove_erase( Container& container, const T& item )
{
	return container.erase( std::remove( std::begin( container ), std::end( container ), item ),
							std::end( container ) );
}

template< typename Container, typename Predicate >
void remove_erase_if( Container& container, Predicate predicate )
{
	container.erase( std::remove_if( std::begin( container ), std::end( container ), predicate ),
					 std::end( container ) );
}

}

//==================================================================================================
// C L A S S E S

//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif // UTILS_HPP
