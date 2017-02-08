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
//  You should have received a copy of the GNU General Public License along with this program.
//  If not, see <http://www.gnu.org/licenses/>.
//
//==================================================================================================

#ifndef STRING_HPP
#define STRING_HPP

//==================================================================================================
// I N C L U D E   F I L E S

#include "Types.h"
#include "Macros.hpp"
#include "CLException.h"

#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>


//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

namespace string
{

	bool begins_with( const std::string& str, const std::string& begining );

	bool ends_with( std::string const& str, std::string const& ending );

	bool contains( std::string const& str, std::string const& tofind );

	bool contains( std::wstring const& str, std::wstring const& tofind );

///
	template< typename T >
	std::string to_string_with_precision( const T a_value, const size_t n = 6 )
	{
		std::ostringstream out;
		out << std::setprecision( static_cast<int32_t>(n) ) << a_value;
		return out.str();
	}

/// Converts a given string into longlong or ulonglong.
	uint64_t to_uint64( const std::string& str );

/// Converts a given string into longlong or ulonglong.
	int64_t to_int64( const std::string& str );

/// Converts a given string into longlong or ulonglong.
	uint32_t to_uint32( const std::string& str );

/// Converts a given string into longlong or ulonglong.
	int32_t to_int32( const std::string& str );

	float to_float( const std::string& str );

	double to_double( const std::string& str );

/// Converts the string into all lower-case letters
	template< class StringType >
	inline void to_lower( StringType& str )
	{
		std::transform( str.begin(), str.end(), str.begin(), ::tolower );
	}

/// Converts the string into all upper-case letters
	template< class StringType >
	inline void to_upper( StringType& str )
	{
		std::transform( str.begin(), str.end(), str.begin(), ::toupper );
	}

/// Converts the string into an array of commands.
	void ToCommand( std::vector< char* >& argv, const std::string& str );

	std::vector< char* > ToCommand( const std::string& str );

	std::vector< std::string > tokenize( const std::string& src, const char delimiter,
										 bool keepEmpty = false );

	bool is_number( const std::string& str );

	bool is_number( const char str );

	bool is_hex( const std::string& str );

	bool is_hex( const char str );

	template void to_lower< std::string >( std::string& str );

	template void to_upper< std::string >( std::string& str );

	template void to_lower< std::wstring >( std::wstring& wstr );

	template void to_upper< std::wstring >( std::wstring& wstr );

	std::string wstring_to_string( const std::wstring& str );

	std::wstring string_to_wstring( const std::string& str );

	std::string valist_to_string( const char* fmt, va_list args );

	template< class T, size_t N >
	std::array< T, N > to_std_array( const std::string& s )
	{
		if( s.size() <= N )
		{
			throw cl::Exception( "array should be big enough for the string", CL_ORIGIN );
		}
		std::array< T, N > d;
		using std::begin; using std::end;
		std::copy( begin( s ), end( s ), begin( d ) );
		return d;
	}

	template< typename T >
	std::string int_to_hex( const T i )
	{
		std::stringstream stream;
		stream << std::showbase << std::setfill( '0' ) << std::setw(
				static_cast<int32_t>(sizeof( T )) * 2 ) << std::hex << i;
		return stream.str();
	}

// trim from start
	static inline std::string& ltrim( std::string& s )
	{
		s.erase( s.begin(), std::find_if( s.begin(), s.end(),
										  std::not1( std::ptr_fun< int, int >( std::isspace ) ) ) );
		return s;
	}

// trim from end
	static inline std::string& rtrim( std::string& s )
	{
		s.erase( std::find_if( s.rbegin(), s.rend(),
							   std::not1( std::ptr_fun< int, int >( std::isspace ) ) ).base(),
				 s.end() );
		return s;
	}

// trim from both ends
	static inline std::string& trim( std::string& s )
	{
		return ltrim( rtrim( s ) );
	}

	class str_const
	{
	public:
		template< std::size_t N >
		constexpr str_const( const char(& a)[N] )
				: p_( a )
				, sz_( N - 1 )
		{ }

		constexpr char operator[]( std::size_t n );

		constexpr std::size_t size();

	private:
		const char* const p_;
		const std::size_t sz_;
	};

}


//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif
