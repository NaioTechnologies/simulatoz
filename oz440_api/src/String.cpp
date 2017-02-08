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

//==================================================================================================
// I N C L U D E   F I L E S

#include "../include/vitals/Print.hpp"
#include "../include/vitals/String.hpp"
#include <regex>


//==================================================================================================
// C O N S T A N T S

static const std::regex
		number( "((\\+|-)?[[:digit:]]+)?(\\.(([[:digit:]]+)?))?((e|E)((\\+|-)?)[[:digit:]]+)?" );

static const std::regex hex( "(0x)?[0-9a-fA-F]+" );

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
uint64_t
string::to_uint64( const std::string& str )
{
	return std::stoul( str );
}

//--------------------------------------------------------------------------------------------------
//
int64_t
string::to_int64( const std::string& str )
{
	return std::stol( str );
}

//--------------------------------------------------------------------------------------------------
//
uint32_t
string::to_uint32( const std::string& str )
{
	return static_cast<uint32_t>( to_uint64( str ) );
}

//--------------------------------------------------------------------------------------------------
//
int32_t
string::to_int32( const std::string& str )
{
	return static_cast<int32_t>( to_int64( str ) );
}

float
string::to_float( const std::string& str )
{
	return std::stof( str );
}

//--------------------------------------------------------------------------------------------------
//
double
string::to_double( const std::string& str )
{
	return std::stod( str );
}

//--------------------------------------------------------------------------------------------------
//
bool
string::begins_with( const std::string& str, const std::string& begining )
{
	return begining.size() <= str.size() &&
		   std::equal( begining.begin(), begining.end(), str.begin() );
}

//--------------------------------------------------------------------------------------------------
//
bool
string::ends_with( std::string const& str, std::string const& ending )
{
	return ending.size() <= str.size() &&
		   std::equal( ending.rbegin(), ending.rend(), str.rbegin() );
}

//--------------------------------------------------------------------------------------------------
//
bool
string::contains( std::string const& str, std::string const& tofind )
{
	return str.find( tofind ) != std::string::npos;
}

//--------------------------------------------------------------------------------------------------
//
bool
string::contains( std::wstring const& str, std::wstring const& tofind )
{
	return str.find( tofind ) != std::wstring::npos;
}

//--------------------------------------------------------------------------------------------------
//
void
string::ToCommand( std::vector<char*>& argv, const std::string& str )
{
	if( !str.empty() )
	{
		size_t iPos = 0;
		size_t pos = str.find( " " );
		std::string sub( "" );

		while( pos != std::string::npos )
		{
			sub = str.substr( iPos, pos - iPos + 1 );
			iPos = pos + 1;
			pos = str.find( " ", iPos );
			argv.push_back( const_cast<char*>(sub.c_str()) );
		}

		sub = str.substr( iPos, std::min( pos, str.size() ) - iPos + 1 );
		argv.push_back( const_cast<char*>(sub.c_str()) );
	}
	argv.push_back( nullptr );
}

//--------------------------------------------------------------------------------------------------
//
std::vector< char* >
string::ToCommand( const std::string& str )
{
	std::vector< char* > argv;
	ToCommand( argv, str );
	return argv;
}

//--------------------------------------------------------------------------------------------------
//
std::vector< std::string >
string::tokenize( const std::string& src, const char delimiter, bool keepEmpty )
{
	std::vector< std::string > results;

	size_t prev{ };
	size_t next{ };

	while( (next = src.find_first_of( delimiter, prev )) != std::string::npos )
	{
		if( keepEmpty or (next - prev != 0) )
		{
			results.push_back( src.substr( prev, next - prev ) );
		}
		prev = next + 1;
	}

	if( prev < src.size() )
	{
		results.push_back( src.substr( prev ) );
	}

	return results;
}

//--------------------------------------------------------------------------------------------------
//
bool
string::is_number( const std::string& s )
{
	return std::regex_match( s, number );
}

//--------------------------------------------------------------------------------------------------
//
bool
string::is_number( const char s )
{
	return std::regex_match( std::string( 1, s ), number );
}

//--------------------------------------------------------------------------------------------------
//
bool
string::is_hex( const std::string& s )
{
	return std::regex_match( s, hex );
}

//--------------------------------------------------------------------------------------------------
//
bool
string::is_hex( const char s )
{
	return std::regex_match( std::string( 1, s ), hex );
}

//--------------------------------------------------------------------------------------------------
//
std::string
string::wstring_to_string( const std::wstring& str )
{
	return std::string( str.begin(), str.end() );
}

//--------------------------------------------------------------------------------------------------
//
std::wstring
string::string_to_wstring( const std::string& str )
{
	return std::wstring( str.begin(), str.end() );
}

//--------------------------------------------------------------------------------------------------
//
std::string
string::valist_to_string( const char* fmt, va_list args )
{
	std::string s{ };
	s.resize( 512 );

	int32_t bytesWritten = vsnprintf( &s[0], s.size(), fmt, args );

	if( bytesWritten == C_API_ERR )
	{
		cl::SystemError( errno, std::generic_category(), CL_ORIGIN );
	}

	s.resize( static_cast<size_t>(bytesWritten) );

	return s;
}

//--------------------------------------------------------------------------------------------------
//
constexpr char
string::str_const::operator[]( std::size_t n )
{
	return n < sz_ ? p_[n] :
		   throw std::out_of_range( "out of bound access to string" );
}

//--------------------------------------------------------------------------------------------------
//
constexpr std::size_t
string::str_const::size()
{
	return sz_;
}

//
////-----------------------------------------------------------------------------
////-----------------------------------------------------------------------------
////---------------------------- CLStringUT class -------------------------------
////-----------------------------------------------------------------------------
////-----------------------------------------------------------------------------
//
//#ifdef DEBUG
//
//#include "TCUnitTest.h"
//
//
////==================================================================================================
//// U N I T   T E S T S   C O D E   S E C T I O N
//
//TC_DEFINE_UNIT_TEST( CLStringUT )
//	{
//		TC_TEST_DIE( !string::is_number( "a" ));
//		TC_TEST_DIE( string::is_number( "1" ));
//		TC_TEST_DIE( string::is_number( "01" ));
//		TC_TEST_DIE( string::is_number( "1.1" ));
//		TC_TEST_DIE( string::is_number( "+1.00" ));
//		TC_TEST_DIE( string::is_number( "-1.46" ));
//		TC_TEST_DIE( string::is_number( "1e+1" ));
//		TC_TEST_DIE( string::is_number( "1E+1" ));
//		TC_TEST_DIE( string::is_number( "1.2E+1" ));
//		TC_TEST_DIE( string::is_number( "-1.2E-1" ));
//		TC_TEST_DIE( string::is_number( "+1.2E-1" ));
//		TC_TEST_DIE( string::is_number( "1e-1" ));
//		TC_TEST_DIE( string::is_number( "1E-1" ));
//		TC_TEST_DIE( string::is_number(
//				"456899054954589048509283409583094566025.98572698758249687592879829485e983445" ) );
//		TC_TEST_DIE(
//				string::is_number( "00000134523.2359872938700000000e-000000009736423976000000" ));
//
//		return true;
//	}
//TC_END_UNIT_TEST( CLStringUT )
//
//#endif
