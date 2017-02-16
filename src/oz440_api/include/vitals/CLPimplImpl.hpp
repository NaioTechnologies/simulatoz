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

#ifndef CLPIMPLIMPL_HPP
#define CLPIMPLIMPL_HPP

//==================================================================================================
// I N C L U D E   F I L E S

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

//==================================================================================================
// T E M P L A T E S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
template< typename T >
cl::pimpl< T >::pimpl()
	: m_{ std::make_unique<T>() }
{ }

//--------------------------------------------------------------------------------------------------
//
template< typename T >
template< typename ...Args >
cl::pimpl< T >::pimpl( Args&& ...args )
	: m_{ std::make_unique<T>( std::forward<Args>( args )... ) }
{ }

//--------------------------------------------------------------------------------------------------
//
template< typename T >
cl::pimpl< T >::pimpl( const T& other )
	: m_( new T( *other.impl_ ) )
{ }

//--------------------------------------------------------------------------------------------------
//
template< typename T >
cl::pimpl< T >::pimpl( pimpl&& other )
	: m_{ std::move( other.m_ ) }
{ }

//--------------------------------------------------------------------------------------------------
//
template< typename T >
cl::pimpl< T >::~pimpl()
{ }

//--------------------------------------------------------------------------------------------------
//
template< typename T >
T*
cl::pimpl< T >::operator->() const
{
	return m_.get();
}

//--------------------------------------------------------------------------------------------------
//
template< typename T >
T&
cl::pimpl< T >::operator*() const
{
	return *m_.get();
}

//--------------------------------------------------------------------------------------------------
//
template< typename T >
void
cl::pimpl< T >::swap( pimpl& other )
{
	m_.swap( other.m_ );
}

//==================================================================================================
// I N L I N E   F U N C T I O N S   C O D E   S E C T I O N

#endif  // CLPIMPLIMPL_HPP
