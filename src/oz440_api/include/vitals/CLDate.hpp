//==================================================================================================
//
//  Copyright(c) 2006 - 2011 Martin Girard, Mathieu Larose, 2012 - 2015 Jean Inderchit
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

#ifndef CLDATE_HPP
#define CLDATE_HPP

//==================================================================================================
// I N C L U D E   F I L E S   A N D   F O R W A R D   D E C L A R A T I O N S

#include "CLPimpl.hpp"

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

namespace datetime
{

///-------------------------------------------------------------------------------------------------
///
/// Class used to hold the system date/time and manipulate the date/time.
///
class Date
{
//--Methods-----------------------------------------------------------------------------------------
public:
	explicit Date( const int64_t dateInSec = -1, const bool ignoreTime = false );

	explicit Date( const uint32_t year, const uint32_t month, const uint32_t day,
				   const uint32_t hour, const uint32_t minute, const uint32_t second );

	Date( const Date& other );

	Date( Date&& other );

	~Date();

	Date& operator=( Date other );

	/// Comparison operators.
	bool operator==( const Date& rhs ) const;

	bool operator<( const Date& rhs ) const;

	bool operator<=( const Date& rhs ) const;

	bool operator>( const Date& rhs ) const;

	bool operator>=( const Date& rhs ) const;

	bool operator!=( const Date& rhs ) const;

	/// Gives the date in seconds from Epoch ( 1970-01-01 00:00:00 ).
	uint64_t get_date_in_seconds() const;

	/// Returns the current date/time in format "YYYY-MM-DD HH:MM:SS"
	void get_date_and_time( std::wstring& outDateTime ) const;

	void get_date_and_time( std::string& outDateTime ) const;

	void get_date_time_subsec( std::string& outDateTime ) const;

	void get_date_time_subsec( std::string& outDateTime, uint32_t& subsec ) const;

	/// "WEEKDAY DD MONTHNAME YYYY HH:MM:SS GMTOFFSET"
		void get_date_and_time_mime( std::string& dateMIMEFormat );

	/// Date is formatted like YYYYMMDD. When cleanTime=true, (hh, mm, ss, ms) are set to 0.
	bool set_date( const std::string& Date, bool cleanTime = false );

	bool set_date( const std::wstring& date, bool cleanTime = false );

	std::wstring get_date() const;

	/// Time is formatted like HHMMSS
	bool set_time( const std::string& time );

	bool set_time( const std::wstring& time );

	std::wstring get_time() const;

	/// Returns the day of the week corresponding to the date
	/// Sunday is 0 and Saturday is 6
	uint64_t get_day_of_week() const;

	/// Add a number of days to the current date
	void add_days( uint32_t daysToAdd );

	/// Computes if the give year is bisextile. year range is between 1900
	/// and 10000
	bool is_bissextile( int32_t year ) const;

	friend void swap( Date& first, Date& second )
	{
		std::swap( first, second );
	}

//--Data members------------------------------------------------------------------------------------
private:
	class Impl;

	cl::pimpl< Impl > pimpl_;
};

namespace date
{
uint64_t get_epoch_in_us();

uint64_t get_epoch_in_ms();

uint64_t get_epoch_in_s();

}

} // cl


//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif // CLDATE_HPP
