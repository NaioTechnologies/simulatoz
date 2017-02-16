//==================================================================================================
//
//  Copyright(c)  2013  Jean Inderchit
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

#ifndef COPY_PRINT_HPP
#define COPY_PRINT_HPP

//==================================================================================================
// I N C L U D E   F I L E S   A N D   F O R W A R D   D E C L A R A T I O N S

#include <iostream>
#include <mutex>
#include <sstream>

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

namespace util
{

//--------------------------------------------------------------------------------------------------
//
template< class S >
void format_line_impl( S& )
{ }

//--------------------------------------------------------------------------------------------------
//
template< class S, class H, class... T >
void format_line_impl( S& stream, H&& head, T&& ... tail )
{
	stream << head;
	format_line_impl( stream, std::forward< T >( tail )... );
}

//--------------------------------------------------------------------------------------------------
//
template<>
void format_line_impl( std::ostream& );

//--------------------------------------------------------------------------------------------------
//
template< class H, class... T >
void format_line_impl( std::ostream& stream, H&& head, T&& ... tail )
{
	stream << head;
	format_line_impl( stream, std::forward< T >( tail )... );
}

//--------------------------------------------------------------------------------------------------
//
template< class... Args >
inline void println( Args&& ... args )
{
	format_line_impl( std::cout, std::forward< Args >( args )... );
	std::cout << std::endl;
}

//--------------------------------------------------------------------------------------------------
//
template< class S >
void format_line_sp_impl( S& )
{ }

//--------------------------------------------------------------------------------------------------
//
template< class S, class H, class... T >
void format_line_sp_impl( S& stream, H&& head, T&& ... tail )
{
	stream << head << " ";
	format_line_sp_impl( stream, std::forward< T >( tail )... );
}

//--------------------------------------------------------------------------------------------------
//
template<>
void format_line_sp_impl( std::ostream& );

//--------------------------------------------------------------------------------------------------
//
template< class H, class... T >
void format_line_sp_impl( std::ostream& stream, H&& head, T&& ... tail )
{
	stream << head << " ";
	format_line_sp_impl( stream, std::forward< T >( tail )... );
}

//--------------------------------------------------------------------------------------------------
//
template< class... Args >
void println_sp( Args&& ... args )
{
	format_line_sp_impl( std::cout, std::forward< Args >( args )... );
	std::cout << std::endl;
}

//--------------------------------------------------------------------------------------------------
//
template< class... Args >
void sync_println( Args&& ... args )
{
	std::mutex mutex;
	std::lock_guard< std::mutex > lock{ mutex };
	format_line_impl( std::cout, std::forward< Args >( args )... );
}

//--------------------------------------------------------------------------------------------------
//
template< class... Args >
void sync_println_sp( Args&& ... args )
{
	std::mutex mutex;
	std::lock_guard< std::mutex > lock{ mutex };
	format_line_sp_impl( std::cout, std::forward< Args >( args )... );
	std::cout << std::endl;
}

//--------------------------------------------------------------------------------------------------
//
template<>
void format_line_impl( std::string& );

//--------------------------------------------------------------------------------------------------
//
template< class H, class... T >
void format_line_impl( std::string& string, H&& head, T&& ... tail )
{
	string << head;
	format_line_impl( string, std::forward< T >( tail )... );
}

//--------------------------------------------------------------------------------------------------
//
template< class... Args >
std::string concat_string( Args&& ... args )
{
	std::stringstream ss;
	format_line_impl( ss, std::forward< Args >( args )... );
	return ss.str();
}

//--------------------------------------------------------------------------------------------------
//
template< typename T/*template< typename, typename... > class ContainerType,
											typename ValueType, typename... Args*/ >
void print_container( const T/*ContainerType< ValueType, Args... >*/& c )
{
	size_t index{ };
	std::cout << "Container with " << c.size() << " objects" << std::endl;
	for( const auto& v : c )
	{
		std::cout << "[" << index << "] " << v << std::endl;
		++index;
	}
	std::cout << std::endl;
}

//--------------------------------------------------------------------------------------------------
// Implement << for pairs: this is needed to print out mappings where range
// iteration goes over (key, value) pairs.
template< typename T, typename U >
std::ostream& operator<<( std::ostream& out, const std::pair< T, U >& p )
{
	out << "[" << p.first << ", " << p.second << "]";
	return out;
}

//--------------------------------------------------------------------------------------------------
//
template< typename T >
inline auto num( T&& t )
{
	return +t;
}

}

//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif // PRINT_HPP
