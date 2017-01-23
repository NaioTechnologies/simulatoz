//==================================================================================================
//
//  Copyright(c)  2013 - 2015 Jean Inderchit
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

#ifndef THREADSAFEQUEUE_HPP
#define THREADSAFEQUEUE_HPP

//==================================================================================================
// I N C L U D E   F I L E S

#include <mutex>
#include <condition_variable>
#include <queue>
#include <memory>


//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

namespace concurrency
{

template< typename T >
class ThreadsafeQueue
{
	typedef std::lock_guard< std::mutex > MutexLockGuard;

//--Methods-----------------------------------------------------------------------------------------
public:
	ThreadsafeQueue();

	ThreadsafeQueue( ThreadsafeQueue const& other );

	void emplace( T&& new_value );

	void push( T&& new_value );

	void wait_and_pop( T& value );

	std::shared_ptr< T > wait_and_pop();

	bool try_pop( T& value );

	std::shared_ptr< T > try_pop();

	size_t size() const;

	bool empty() const;

//--Data members------------------------------------------------------------------------------------
private:
	mutable std::mutex queueAccess_;
	std::queue< T > dataQueue_;
	std::condition_variable dataCond_;
};

}

//==================================================================================================
// T E M P L A T E S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
template< class T >
concurrency::ThreadsafeQueue< T >::ThreadsafeQueue()
{ }

//--------------------------------------------------------------------------------------------------
//
template< class T >
concurrency::ThreadsafeQueue< T >::ThreadsafeQueue( const ThreadsafeQueue& other )
{
	MutexLockGuard lk( other.queueAccess_ );
	std::swap( dataQueue_, other.dataQueue_ );
}

//--------------------------------------------------------------------------------------------------
//
template< class T >
void
concurrency::ThreadsafeQueue< T >::emplace( T&& new_value )
{
	MutexLockGuard lk( queueAccess_ );
	dataQueue_.emplace( std::forward< T >( new_value ) );
	dataCond_.notify_one();
}

//--------------------------------------------------------------------------------------------------
//
template< class T >
void
concurrency::ThreadsafeQueue< T >::push( T&& new_value )
{
	MutexLockGuard lk( queueAccess_ );
	dataQueue_.push( std::forward< T >( new_value ) );
	dataCond_.notify_one();
}

//--------------------------------------------------------------------------------------------------
//
template< class T >
void
concurrency::ThreadsafeQueue< T >::wait_and_pop( T& value )
{
	std::unique_lock< std::mutex > lk( queueAccess_ );
	dataCond_.wait( lk, [ this ]
	{
	    return !dataQueue_.empty();
	} );

	value = std::move( dataQueue_.front() );

	dataQueue_.pop();
}

//--------------------------------------------------------------------------------------------------
//
template< class T >
std::shared_ptr< T >
concurrency::ThreadsafeQueue< T >::wait_and_pop()
{
	std::unique_lock< std::mutex > lk( queueAccess_ );
	dataCond_.wait( lk, [ this ]
	{
	    return !dataQueue_.empty();
	} );

	std::shared_ptr< T > res( std::make_shared< T >( dataQueue_.front() ) );
	dataQueue_.pop();
	return res;
}

//--------------------------------------------------------------------------------------------------
//
template< class T >
bool
concurrency::ThreadsafeQueue< T >::try_pop( T& value )
{
	MutexLockGuard lk( queueAccess_ );

	if( dataQueue_.empty() )
	{
		return false;
	}

	value = std::move( dataQueue_.front() );
	dataQueue_.pop();
	return true;
}

//--------------------------------------------------------------------------------------------------
//
template< class T >
std::shared_ptr< T >
concurrency::ThreadsafeQueue< T >::try_pop()
{
	MutexLockGuard lk( queueAccess_ );
	if( dataQueue_.empty() )
	{
		return std::shared_ptr< T >();
	}

	std::shared_ptr< T > res( std::make_shared< T >( dataQueue_.front() ) );
	dataQueue_.pop();
	return res;
}

//--------------------------------------------------------------------------------------------------
//
template< class T >
size_t
concurrency::ThreadsafeQueue< T >::size() const
{
	MutexLockGuard lk( queueAccess_ );
	return dataQueue_.size();
}

//--------------------------------------------------------------------------------------------------
//
template< class T >
bool
concurrency::ThreadsafeQueue< T >::empty() const
{
	MutexLockGuard lk( queueAccess_ );
	return dataQueue_.empty();
}


//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif  // THREADSAFEQUEUE_HPP
