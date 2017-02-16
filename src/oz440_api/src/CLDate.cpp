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

//==================================================================================================
// I N C L U D E   F I L E S

#include "vitals/CLDate.hpp"
#include "vitals/Macros.hpp"
#include "vitals/Format.hpp"
#include "vitals/String.hpp"
#include "vitals/CLPimplImpl.hpp"

#include <chrono>
#include <sys/time.h>

//==================================================================================================
// P I M P L   D E F I N I T I O N   C O D E   S E C T I O N

class datetime::Date::Impl
{
//--Methods-----------------------------------------------------------------------------------------
public:
	explicit Impl( const int64_t dateInSec, const bool ignoreTime );

	explicit Impl( const uint32_t year, const uint32_t month, const uint32_t day,
				   const uint32_t hour, const uint32_t minutes, const uint32_t seconds );

	Impl( const Impl& other );

	Impl( Impl&& other );

	~Impl();

	Impl& operator=( Impl other );

	tm& get_tm();

	const uint32_t& get_us() const;

	const std::time_t& get_time_t() const;

	void set_tm( tm* systime );

	const std::array< uint32_t, 12 >& get_days_of_month();

	void swap( Impl& first, Impl& second )
	{
		using std::swap;
		swap( first.time_, second.time_ );
		swap( first.sysTime_, second.sysTime_ );
	}

	//--Data members------------------------------------------------------------------------------------
private:
	// Data members
	static const uint32_t startYear_;
	static const std::array< uint32_t, 12 > daysOfMonth_;

	std::tm* sysTime_;
	std::time_t time_;
	uint32_t us_;
};

//==================================================================================================
// P I M P L   I M P L E M E N T A T I O N   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Impl::Impl( const int64_t dateInSec, const bool ignoreTime )
	: sysTime_{ }
	, time_{ }
	, us_{ }
{
	if( dateInSec > -1 )
	{
		time_ = static_cast<time_t>(dateInSec);
	}
	else
	{
		timeval tv;
		if( gettimeofday( &tv, nullptr ) == C_API_ERR )
		{
			throw cl::SystemError( errno, std::generic_category(), CL_ORIGIN );
		}

		time_ = tv.tv_sec;
		us_ = static_cast<uint32_t>(tv.tv_usec);
	}

	// Transform raw time into local time
	if( (sysTime_ = localtime( &time_ )) == NULL )
	{
		throw cl::Exception( "localtime()", CL_ORIGIN );
	}

	if( ignoreTime )
	{
		// Zeros the time values.
		sysTime_->tm_hour = 0;
		sysTime_->tm_min = 0;
		sysTime_->tm_sec = 0;
	}
}

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Impl::Impl( const uint32_t year, const uint32_t month, const uint32_t day,
					  const uint32_t hour, const uint32_t minute, const uint32_t second )
	: sysTime_{ }
	, time_{ std::time( NULL ) }
	, us_{ }
{
	if( (sysTime_ = localtime( &time_ )) == NULL )
	{
		throw cl::Exception( "localtime()", CL_ORIGIN );
	}

	sysTime_->tm_year = static_cast<int32_t>(year - 1900);
	sysTime_->tm_mon = static_cast<int32_t>(month - 1);
	sysTime_->tm_mday = static_cast<int32_t>(day);
	sysTime_->tm_hour = static_cast<int32_t>(hour);
	sysTime_->tm_min = static_cast<int32_t>(minute);
	sysTime_->tm_sec = static_cast<int32_t>(second);

	time_ = std::mktime( sysTime_ );
}

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Impl::Impl( const Impl& other )
	: sysTime_{ }
	, time_{ other.time_ }
	, us_{ other.us_ }
{
	// Transform raw time into local time
	if( (sysTime_ = localtime( &time_ )) == NULL )
	{
		throw cl::Exception( "localtime()", CL_ORIGIN );
	}
}

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Impl::Impl( Impl&& other )
	: sysTime_{ }
	, time_{ }
	, us_{ }
{
	swap( *this, other );
}

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Impl::~Impl()
{ }

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Impl&
datetime::Date::Impl::operator=( Impl other )
{
	swap( *this, other );
	return *this;
}

//--------------------------------------------------------------------------------------------------
//
tm&
datetime::Date::Impl::get_tm()
{
	return *sysTime_;
}

//--------------------------------------------------------------------------------------------------
//
const uint32_t&
datetime::Date::Impl::get_us() const
{
	return us_;
}

//--------------------------------------------------------------------------------------------------
//
const std::time_t&
datetime::Date::Impl::get_time_t() const
{
	return time_;
}

//--------------------------------------------------------------------------------------------------
//
void
datetime::Date::Impl::set_tm( tm* systime )
{
	sysTime_ = systime;
}

//--------------------------------------------------------------------------------------------------
//
const std::array< uint32_t, 12 >&
datetime::Date::Impl::get_days_of_month()
{
	return daysOfMonth_;
}


//==================================================================================================
// C O N S T A N T (S)   C O D E   S E C T I O N

const uint32_t datetime::Date::Impl::startYear_ = 1970;

const std::array< uint32_t, 12 > datetime::Date::Impl::daysOfMonth_{
	{ 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 } };


//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Date( const int64_t dateInSec, const bool ignoreTime )
	: pimpl_{ dateInSec, ignoreTime }
{ }

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Date( const uint32_t year, const uint32_t month, const uint32_t day, const uint32_t hour,
				const uint32_t minute, const uint32_t second )
	: pimpl_{ year, month, day, hour, minute, second }
{ }

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Date( const Date& other )
	: pimpl_{ *other.pimpl_ }
{ }

//--------------------------------------------------------------------------------------------------
//
datetime::Date::Date( Date&& other )
	: pimpl_{ std::move( other.pimpl_ ) }
{ }

//--------------------------------------------------------------------------------------------------
//
datetime::Date::~Date()
{ }

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
datetime::Date&
datetime::Date::operator=( Date other )
{
	swap( *this, other );
	return *this;
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::operator==( const Date& other ) const
{
	return get_date_in_seconds() == other.get_date_in_seconds();
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::operator<=( const Date& other ) const
{
	return get_date_in_seconds() <= other.get_date_in_seconds();
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::operator>=( const Date& other ) const
{
	return get_date_in_seconds() >= other.get_date_in_seconds();
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::operator<( const Date& other ) const
{
	return get_date_in_seconds() < other.get_date_in_seconds();
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::operator>( const Date& other ) const
{
	return get_date_in_seconds() > other.get_date_in_seconds();
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::operator!=( const Date& other ) const
{
	return get_date_in_seconds() != other.get_date_in_seconds();
}

//--------------------------------------------------------------------------------------------------
//
uint64_t
datetime::Date::get_date_in_seconds() const
{
	return static_cast<uint64_t>( pimpl_->get_time_t() );
}

//--------------------------------------------------------------------------------------------------
//
void
datetime::Date::get_date_and_time( std::wstring& dateAndTime ) const
{
	std::string tmp;
	get_date_and_time( tmp );
	dateAndTime = string::string_to_wstring( tmp );
}

//--------------------------------------------------------------------------------------------------
//
void
datetime::Date::get_date_and_time( std::string& dateAndTime ) const
{
	dateAndTime = tfm::format( "%04i-%02i-%02i %02i:%02i:%02i",
							   pimpl_->get_tm().tm_year + 1900,
							   pimpl_->get_tm().tm_mon + 1,
							   pimpl_->get_tm().tm_mday,
							   pimpl_->get_tm().tm_hour,
							   pimpl_->get_tm().tm_min,
							   pimpl_->get_tm().tm_sec );
}

//--------------------------------------------------------------------------------------------------
//
void
datetime::Date::get_date_time_subsec( std::string& dateAndTime ) const
{
	dateAndTime = tfm::format( "%04i-%02i-%02i %02i:%02i:%02i.%03ld",
							   pimpl_->get_tm().tm_year + 1900,
							   pimpl_->get_tm().tm_mon + 1,
							   pimpl_->get_tm().tm_mday,
							   pimpl_->get_tm().tm_hour,
							   pimpl_->get_tm().tm_min,
							   pimpl_->get_tm().tm_sec,
							   pimpl_->get_us() / 1000 );
}

//--------------------------------------------------------------------------------------------------
//
void
datetime::Date::get_date_time_subsec( std::string& dataAndTime, uint32_t& subsec ) const
{
	get_date_and_time( dataAndTime );
	subsec = pimpl_->get_us() / 1000;
}

//--------------------------------------------------------------------------------------------------
//
void
datetime::Date::get_date_and_time_mime( std::string& dateMIMEFormat )
{
	size_t bufferSize{ 256 };
	char buff[bufferSize];
	std::string format{ "%F %T %z" };

	size_t bytesRead = std::strftime( buff, bufferSize, format.c_str(), &pimpl_->get_tm() );

	if( !bytesRead )
	{
		throw cl::Exception( "count was reached before the entire string could be stored",
							 CL_ORIGIN );
	}

	dateMIMEFormat = std::string( buff, bytesRead );
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::set_date( const std::string& date, bool cleanTime )
{
	bool isValid = true;

	int32_t year = string::to_int32( date.substr( 0, 4 ).c_str() );
	int32_t month = string::to_int32( date.substr( 4, 2 ).c_str() );
	int32_t day = string::to_int32( date.substr( 6, 2 ).c_str() );

	if( year < 2000 || month < 1 || month > 12 || day < 1 || day > 31 ||
		date.length() != 8 )
	{
		isValid = false;
	}
	else
	{
		pimpl_->get_tm().tm_year = year;
		pimpl_->get_tm().tm_mon = month;
		pimpl_->get_tm().tm_mday = day;

		if( cleanTime )
		{
			// Zeros the time values.
			pimpl_->get_tm().tm_hour = 0;
			pimpl_->get_tm().tm_min = 0;
			pimpl_->get_tm().tm_sec = 0;
		}

		std::time_t time;

		if( (time = mktime( &pimpl_->get_tm() )) == C_API_ERR )
		{
			throw cl::SystemError( errno, std::generic_category(), CL_ORIGIN );
		}

		tm* tmp;
		if( (tmp = localtime( &time )) == NULL )
		{
			throw cl::SystemError( errno, std::generic_category(), CL_ORIGIN );
		}
		pimpl_->set_tm( tmp );
	}
	return isValid;
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::set_date( const std::wstring& date, bool cleanTime )
{
	return set_date( string::wstring_to_string( date ), cleanTime );
}

//--------------------------------------------------------------------------------------------------
//
std::wstring
datetime::Date::get_date() const
{
	std::wstring date;
	std::wstringstream ws;

	ws << pimpl_->get_tm().tm_year;
	date = ws.str();
	ws.str( L"" );

	ws << pimpl_->get_tm().tm_mon;
	date += ws.str();
	ws.str( L"" );

	ws << pimpl_->get_tm().tm_mday;
	date += ws.str();
	ws.str( L"" );

	return date;
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::set_time( const std::string& time )
{
	bool isValid = true;

	int hour = string::to_int32( time.substr( 0, 2 ).c_str() );
	int minute = string::to_int32( time.substr( 2, 2 ).c_str() );
	int second = 0;

	if( time.length() == 3 )
	{
		hour = atoi( time.substr( 0, 1 ).c_str() );
		minute = atoi( time.substr( 1, 2 ).c_str() );
	}
	else if( time.length() == 6 )
	{
		second = atoi( time.substr( 4, 2 ).c_str() );
	}
	if( time.length() != 6 && time.length() != 4 && time.length() != 3 )
	{
		isValid = false;
	}
	else if( hour < 0 || minute < 0 || second < 0 || hour > 24 || minute > 60 ||
			 second > 60 )
	{
		isValid = false;
	}
	else
	{
		pimpl_->get_tm().tm_hour = hour;
		pimpl_->get_tm().tm_min = minute;
		pimpl_->get_tm().tm_sec = second;
	}

	std::time_t t;

	if( (t = mktime( &pimpl_->get_tm() )) == C_API_ERR )
	{
		throw cl::SystemError( errno, std::generic_category(), CL_ORIGIN );
	}

	tm* tmp;

	if( (tmp = localtime( &t )) == NULL )
	{
		throw cl::SystemError( errno, std::generic_category(), CL_ORIGIN );
	}

	pimpl_->set_tm( tmp );

	return isValid;
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::set_time( const std::wstring& time )
{
	return set_time( string::wstring_to_string( time ));
}

//--------------------------------------------------------------------------------------------------
//
std::wstring
datetime::Date::get_time() const
{
	std::wstring time;
	std::wstringstream ws;

	ws << pimpl_->get_tm().tm_hour;
	time = ws.str();
	ws.str( L"" );

	ws << pimpl_->get_tm().tm_min;
	time += ws.str();
	ws.str( L"" );

	ws << pimpl_->get_tm().tm_sec;
	time += ws.str();
	ws.str( L"" );

	return time;
}

//--------------------------------------------------------------------------------------------------
//
uint64_t
datetime::Date::get_day_of_week() const
{
	return static_cast<uint64_t>( pimpl_->get_tm().tm_wday );
}

//--------------------------------------------------------------------------------------------------
//
void
datetime::Date::add_days( uint32_t daysToAdd )
{
	// the variable month is used because it has to reach the value 13
	// which is impossible for sysTime_->tm_mon
	uint month = static_cast<uint>( pimpl_->get_tm().tm_mon );
	uint day = static_cast<uint>( pimpl_->get_tm().tm_mday );
	uint bissexDays = 0;

	(month == 2 && is_bissextile( pimpl_->get_tm().tm_year )) ? bissexDays = 1 : bissexDays = 0;

	while( day + daysToAdd > (pimpl_->get_days_of_month()[month - 1] + bissexDays) )
	{
		daysToAdd -= (pimpl_->get_days_of_month()[month - 1] + bissexDays - day);
		day = 0;

		month++;
		(month == 2 && is_bissextile( pimpl_->get_tm().tm_year )) ? bissexDays = 1 : bissexDays = 0;

		if( month > 12 )
		{
			month = 1;
			pimpl_->get_tm().tm_year++;
		}
	}

	pimpl_->get_tm().tm_mday = static_cast<int>( day + daysToAdd );
	pimpl_->get_tm().tm_mon = static_cast<int>( month );
}

//--------------------------------------------------------------------------------------------------
//
bool
datetime::Date::is_bissextile( int32_t year ) const
{
	if( year % 100 != 0 )
	{
		return year % 4 == 0;
	}
	else
	{
		return year % 4 == 0 && year % 400 == 0;
	}
}

//--------------------------------------------------------------------------------------------------
//
uint64_t
datetime::date::get_epoch_in_us()
{
	return static_cast<uint64_t>( std::chrono::duration_cast< std::chrono::microseconds >(
		std::chrono::system_clock::now().time_since_epoch() ).count() );
}

//--------------------------------------------------------------------------------------------------
//
uint64_t
datetime::date::get_epoch_in_ms()
{
	return static_cast<uint64_t>( std::chrono::duration_cast< std::chrono::milliseconds >(
		std::chrono::system_clock::now().time_since_epoch() ).count() );
}

//--------------------------------------------------------------------------------------------------
//
uint64_t
datetime::date::get_epoch_in_s()
{
	return static_cast<uint64_t>( std::chrono::duration_cast< std::chrono::seconds >(
		std::chrono::system_clock::now().time_since_epoch() ).count() );
}
//
//
////--------------------------------------------------------------------------------------------------
////--------------------------------------------------------------------------------------------------
////------------------------------------------- UNIT TESTS -------------------------------------------
////--------------------------------------------------------------------------------------------------
////--------------------------------------------------------------------------------------------------
//
////==================================================================================================
//// U N I T   T E S T   C O D E   S E C T I O N
//
//#include "TCUnitTest.h"
//
////--------------------------------------------------------------------------------------------------
////
//TC_DEFINE_UNIT_TEST( CLDateUT )
//	{
//		const uint64_t seconds{ 578'620'800 };
//		datetime::Date dateOne( seconds );
//
//		std::string dateTime;
//		dateOne.get_date_and_time( dateTime );
//		TC_TEST( dateTime.compare( "1988-05-03 02:00:00" ) == 0 );
//		TC_TEST( dateOne.get_date_in_seconds() == seconds );
//
//		datetime::Date dateTwo( dateOne );
//		TC_TEST( dateOne == dateTwo );
//
//		datetime::Date dateThree( std::move( dateOne ) );
//		TC_TEST( dateTwo == dateThree );
//
//		datetime::Date dateFour( 1988, 05, 03, 02, 00, 00 );
//		TC_TEST( dateThree == dateFour );
//
//		dateTwo.add_days( 1 );
//		dateTwo.get_date_and_time( dateTime );
//		TC_TEST( dateTime.compare( "1988-05-04 02:00:00" ) == 0 );
//
//		return true;
//	}
//TC_END_UNIT_TEST( CLDateUT )
