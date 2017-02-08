//==================================================================================================
//
//   Author:    Andy Rushton
//   Copyright: (c) Southampton University 1999-2004
//              (c) Andy Rushton           2004 onwards
//   License:   BSD License, see ../docs/license.html
//
//   This is a portable interface to the file system.
//
//   The idea is that you write all file system access code using these functions,
//   which are ported to all platforms that we are interested in. Therefore your
//   code is inherently portable.
//
//==================================================================================================

//==================================================================================================
// I N C L U D E   F I L E S

#include "../include/vitals/CLFileSystem.h"

#ifdef MSWINDOWS
#include <windows.h>
#include <dos.h>
#include <direct.h>
#include <fcntl.h>
#include <io.h>
#include <sys/types.h>
#include <sys/stat.h>
#else

#include "../include/vitals/CLException.h"
#include "../include/vitals/Macros.hpp"
#include "../include/vitals/Utils.hpp"
#include "../include/vitals/Print.hpp"

#include <functional>

#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <mntent.h>
#include <sys/stat.h>
#include <sys/param.h>
#include <sys/mount.h>

#endif

//==================================================================================================
// F R E E   F U N C T I O N S   C O D E   S E C T I O N

namespace cl
{

namespace filesystem
{

bool
enumerate_files( IterateData& data )
{
	bool aborted{ };
	bool result{ true };

	const std::string path_to_element{ data.srcPath };
	const std::string name_of_element{ data.name };

	EnumData& theData( dynamic_cast<EnumData&>( data ) );
	std::string path{ path_to_element };
	path = cl::filesystem::folder_append_separator( path );
	path.append( name_of_element );

	std::error_code ec;
	theData.visitor->set_directory( cl::filesystem::is_folder( path, ec ) );
	theData.visitor->on_file_found( path_to_element, name_of_element, aborted );

	if( aborted )
	{
		result = false;
	}

	return result;
}

//-------------------------------------------------------------------------------------------------
// Warning: this function can handle a maximum of MAX_OBJECTS in a folder. The function returns
// false if the nb. of objects is greater than MAX_OBJECTS, although the action will be still
// carried on for the first 0-MAX_OBJECTS files.
bool
iterate_elements( const std::string& folder, const std::string& destFolder, ActionFunc fun,
				  IterateData* data, bool recursive )
{
	IterateData localData;
	if( data == NULL )
	{
		data = &localData;
	}

	std::vector< std::string > all_elements = folder_all( folder );

	for( const auto& element : all_elements )
	{
		const std::string new_path = folder_to_path( folder, element );

		std::error_code ec;
		if( !is_symlink_file( new_path, ec ) ) // simply ignore the symlinks
		{
			data->name = element;
			data->destName = element;
			data->srcPath = folder;
			data->destName = element;
			data->destPath = destFolder;
			data->isFolder = false;

			// Apply action to file
			if( !fun( *data ) )
			{
				return false;
			}

			// If it's a directory recurse into it
			if( is_folder( new_path, ec ) && recursive )
			{
				data->srcPath = new_path;
				data->destPath = new_path;
				data->isFolder = true;

				IterateData dataCopy = *data;
				iterate_elements( data->srcPath, data->destPath, fun, data, recursive );
				*data = dataCopy;

				std::string parentSrc = folder_up( data->srcPath, 1 );
				data->srcPath = parentSrc;
			}

			//data->name = element;
			//data->destName = element;
		}
	}
	return true;
}

//-------------------------------------------------------------------------------------------------
//
void
enum_files_recurs( const std::string& folder, EnumVisitor& visitor, const bool recurse,
				   bool& aborted )
{
	if( aborted )
	{
		return;
	}

	std::string pathToFolder{ folder };
	std::string dest{ };

	std::string::size_type pos = pathToFolder.find( "/*" );
	if( pos != std::string::npos )
	{
		pathToFolder.erase( pos );
	}

	EnumData data;
	data.visitor = &visitor;

	if( folder_exists( pathToFolder ) )
	{
		iterate_elements( pathToFolder, dest, enumerate_files, &data, recurse );
	}
}

//-------------------------------------------------------------------------------------------------
//
bool
enum_elements( const std::string& folderPath, EnumVisitor& visitor, bool recurse )
{
	bool aborted{ };
	enum_files_recurs( folderPath, visitor, recurse, aborted );
	return aborted;
}

//-------------------------------------------------------------------------------------------------
//
// function for testing whether a character matches a set I can't remember the exact rules and I
// have no definitive references but: a set contains characters, escaped characters (I think) and
// ranges in the form a-z The character '-' can only appear at the start of the set where it is not
// interpreted as a range. This is a horrible mess - blame the Unix folks for making a hash of
// wildcards first expand any ranges and remove escape characters to make life more palatable
//
static bool match_set( const std::string& set, char match )
{
	std::string simple_set;
	for( std::string::const_iterator i = set.begin(); i != set.end(); ++i )
	{
		switch( *i )
		{
			case '-':
			{
				if( i == set.begin() )
				{
					simple_set += *i;
				}
				else if( i + 1 == set.end() )
				{
					return false;
				}
				else
				{
					// found a set. The first character is already in the result, so first remove
					// it (the set might be empty)
					simple_set.erase( simple_set.end() - 1 );
					char last = *++i;
					for( char ch = *(i - 2); ch <= last; ch++ )
					{
						simple_set += ch;
					}
				}
				break;
			}
			case '\\':
				if( i + 1 == set.end() )
				{
					return false;
				}
				simple_set += *++i;
				break;
			default:
				simple_set += *i;
				break;
		}
	}
	std::string::size_type result = simple_set.find( match );
	return result != std::string::npos;
}

//-------------------------------------------------------------------------------------------------
//
// the recursive bit - basically whenever a * is found you recursively call this for each candidate
// substring match until either it succeeds or you run out of string to match for each * in the
// wildcard another level of recursion is created
//
static bool match_remainder( const std::string& wild, std::string::const_iterator wildi,
							 const std::string& match, std::string::const_iterator matchi )
{
	//cerr << "match_remainder called at " << *matchi << " with wildcard " << *wildi << endl;
	while( wildi != wild.end() && matchi != match.end() )
	{
		//cerr << "trying to match " << *matchi << " with wildcard " << *wildi << endl;
		switch( *wildi )
		{
			case '*':
			{
				++wildi;
				++matchi;
				for( std::string::const_iterator i = matchi; i != match.end(); ++i )
				{
					// deal with * at the end of the wildcard - there is no remainder then
					if( wildi == wild.end() )
					{
						if( i == match.end() - 1 )
						{
							return true;
						}
					}
					else if( match_remainder( wild, wildi, match, i ) )
					{
						return true;
					}
				}
				return false;
			}
			case '[':
			{
				// scan for the end of the set using a similar method for avoiding escaped characters
				bool found = false;
				auto end = wildi + 1;
				for( ; !found && end != wild.cend(); ++end )
				{
					switch( *end )
					{
						case ']':
						{
							// found the set, now match with its contents excluding the brackets
							if( !match_set(
								wild.substr( static_cast<size_t>(wildi - wild.begin() + 1),
											 static_cast<size_t>(end - wildi - 1) ),
								*matchi ) )
							{
								return false;
							}
							found = true;
							break;
						}
						case '\\':
						{
							if( end == wild.end() - 1 )
							{
								return false;
							}
							++end;
							break;
						}
						default:
							break;
					}
				}
				if( !found )
				{
					return false;
				}
				++matchi;
				wildi = end;
				break;
			}
			case '?':
			{
				++wildi;
				++matchi;
				break;
			}
			case '\\':
			{
				if( wildi == wild.end() - 1 )
				{
					return false;
				}
				++wildi;
				if( *wildi != *matchi )
				{
					return false;
				}
				++wildi;
				++matchi;
				break;
			}
			default:
			{
				if( *wildi != *matchi )
				{
					return false;
				}
				++wildi;
				++matchi;
				break;
			}
		}
	}
	bool result = wildi == wild.end() && matchi == match.end();
	return result;
}

//-------------------------------------------------------------------------------------------------
//
// like all recursions the exported function has a simpler interface than the recursive function
// and is just a 'seed' to the recursion itself
//
bool wildcard( const std::string& wild, const std::string& match )
{
	return match_remainder( wild, wild.begin(), match, match.begin() );
}

////////////////////////////////////////////////////////////////////////////////
// definitions of separators

#ifdef MSWINDOWS
static const char* separator_set = "\\/";
static const char preferred_separator = '\\';
#else
static const char* separator_set = "/";
static const char preferred_separator = '/';
#endif

namespace
{

bool is_separator( char ch )
{
	for( int i = 0; separator_set[i]; i++ )
	{
		if( separator_set[i] == ch )
		{
			return true;
		}
	}
	return false;
}

std::string remove_trailing_slash( const std::string& thing )
{
	// strip off any trailing separator because that will cause the stat function to fail
	std::string path = thing;
	if( !path.empty() && is_separator( path[path.size() - 1] ) )
	{
		path.erase( path.size() - 1, 1 );
	}
	return path;
}

void stat( const std::string& path, struct stat& buf, std::error_code& ec )
{
	if( stat( path.c_str(), &buf ) == C_API_ERR )
	{
		ec.assign( errno, std::generic_category() );
	}
}

void lstat( const std::string& path, struct stat& buf, std::error_code& ec )
{
	if( lstat( path.c_str(), &buf ) == C_API_ERR )
	{
		ec.assign( errno, std::generic_category() );
	}
}

}

////////////////////////////////////////////////////////////////////////////////
// implement string comparison of paths - Unix is case-sensitive, Windoze is case-insensitive

#ifdef MSWINDOWS

static std::string lowercase(const std::string& val)
{
  std::string text = val;
  for (unsigned i = 0; i < text.size(); i++)
	text[i] = tolower(text[i]);
  return text;
}

#endif

bool path_compare( const std::string& l, const std::string& r )
{
	#ifdef MSWINDOWS
	return lowercase(l) == lowercase(r);
	#else
	return l == r;
	#endif
}

////////////////////////////////////////////////////////////////////////////////
// Internal data structure used to hold the different parts of a filespec

class file_specification
{
private:
	bool m_relative; // true = relative, false = absolute
	std::string
		m_drive; // drive - drive letter (e.g. "c:") or the path for an UNC (e.g. "\\somewhere")
	//         empty if not known or on Unix
	std::vector< std::string > m_path; // the subdirectory path to follow from the drive
	std::string m_filename;          // the filename
public:
	file_specification()
		: m_relative{ }
	{ }

	~file_specification()
	{ }

	bool initialise_folder( const std::string& spec );

	bool initialise_file( const std::string& spec );

	bool simplify();

	bool make_absolute( const std::string& root = folder_current_full() );

	bool make_absolute( const file_specification& root );

	bool make_relative( const std::string& root = folder_current_full() );

	bool make_relative( const file_specification& root );

	bool relative() const
	{ return m_relative; }

	bool absolute() const
	{ return !relative(); }

	void set_relative()
	{ m_relative = true; }

	void set_absolute()
	{ m_relative = false; }

	const std::string& drive( void ) const
	{ return m_drive; }

	std::string& drive()
	{ return m_drive; }

	void set_drive( const std::string& d )
	{ m_drive = d; }

	const std::vector< std::string >& path() const
	{ return m_path; }

	std::vector< std::string >& path()
	{ return m_path; }

	void set_path( const std::vector< std::string >& p )
	{ m_path = p; }

	void add_subpath( const std::string& subpath )
	{ m_path.push_back( subpath ); }

	unsigned subpath_size( void ) const
	{ return static_cast<unsigned>(m_path.size()); }

	const std::string& subpath_element( unsigned i ) const
	{ return m_path[i]; }

	void subpath_erase( unsigned i )
	{ m_path.erase( m_path.begin() + i ); }

	const std::string& file() const
	{ return m_filename; }

	std::string& file()
	{ return m_filename; }

	void set_file( const std::string& f )
	{ m_filename = f; }

	std::string image( void ) const;
};

bool file_specification::initialise_folder( const std::string& folder_spec )
{
	std::string spec = folder_spec;
	m_relative = true;
	m_drive.erase();
	m_path.clear();
	m_filename.erase();
	unsigned i = 0;
	#ifdef MSWINDOWS
	// first split off the drive letter or UNC prefix on Windows
	if (spec.size() >= 2 && isalpha(spec[0]) && spec[1] == ':')
	{
	  // found a drive letter
	  i = 2;
	  m_drive = spec.substr(0, 2);
	  m_relative = false;
	  // if there is a drive but no path or a relative path, get the current
	  // path for this drive and prepend it to the path
	  if (i == spec.size() || !is_separator(spec[i]))
	  {
		// getdcwd requires the drive number (1..26) not the letter (A..Z)
		char path [MAX_PATH+1];
		int drivenum = toupper(m_drive[0]) - 'A' + 1;
		if (_getdcwd(drivenum, path, MAX_PATH+1))
		{
		  // the path includes the drive so we have the drive info twice
		  // need to prepend this absolute path to the spec such that any remaining relative path is still retained
		  if (!is_separator(path[strlen(path)-1])) spec.insert(2, 1, preferred_separator);
		  spec.insert(2, path+2);
		}
		else
		{
		  // non-existent drive - fill in just the root directory
		  spec.insert(2, 1, preferred_separator);
		}
	  }
	}
	else if (spec.size() >= 2 && is_separator(spec[0]) && is_separator(spec[1]))
	{
	  // found an UNC prefix
	  i = 2;
	  // find the end of the prefix by scanning for the next seperator or the end of the spec
	  while (i < spec.size() && !is_separator(spec[i])) i++;
	  m_drive = spec.substr(0, i);
	  m_relative = false;
	}
	#endif
	#ifdef CYGWIN
	// first split off the drive letter or UNC prefix on Windows - the Cygwin environment supports these too
	if (spec.size() >= 2 && isalpha(spec[0]) && spec[1] == ':')
	{
	  // found a drive letter
	  i = 2;
	  m_drive = spec.substr(0, 2);
	  m_relative = false;
	  // if there is a drive but no path or a relative path, get the current
	  // path for this drive and prepend it to the path
	  if (i == spec.size() || !is_separator(spec[i]))
	  {
		// non-existent drive - fill in just the root directory
		spec.insert(2, 1, preferred_separator);
	  }
	}
	else if (spec.size() >= 2 && is_separator(spec[0]) && is_separator(spec[1]))
	{
	  // found an UNC prefix
	  i = 2;
	  // find the end of the prefix by scanning for the next seperator or the end of the spec
	  while (i < spec.size() && !is_separator(spec[i])) i++;
	  m_drive = spec.substr(0, i);
	  m_relative = false;
	}
	#endif
	// check whether the path is absolute or relative and discard the leading / if absolute
	if( i < spec.size() && is_separator( spec[i] ) )
	{
		m_relative = false;
		i++;
		#ifdef MSWINDOWS
		// if there's no drive, fill it in on Windows since absolute paths must have a drive
		if (m_drive.empty())
		{
		  m_drive += (char)(_getdrive() - 1 + 'A');
		  m_drive += ':';
		}
		#endif
	}
	// now extract the path elements - note that a trailing / is not significant since /a/b/c/ === /a/b/c
	// also note that the leading / has been discarded - all paths are relative
	// if absolute() is set, then paths are relative to the drive, else they are relative to the current path
	unsigned start = i;
	while( i <= spec.size() )
	{
		if( i == spec.size() )
		{
			// path element terminated by the end of the string
			// discard this element if it is zero length because that represents the trailing /
			if( i != start )
			{
				m_path.push_back( spec.substr( start, i - start ) );
			}
		}
		else if( is_separator( spec[i] ) )
		{
			// path element terminated by a separator
			m_path.push_back( spec.substr( start, i - start ) );
			start = i + 1;
		}
		i++;
	}
	// TODO - some error handling?
	return true;
}

bool file_specification::initialise_file( const std::string& spec )
{
	m_filename.erase();
	// remove last element as the file and then treat the rest as a folder
	size_t i = spec.size();
	while( --i )
	{
		if( is_separator( spec[i] ) )
		{
			break;
		}
		#ifdef MSWINDOWS
		// on windoze you can say a:fred.txt so the colon separates the path from the filename
		else if (i == 1 && spec[i] == ':')
		  break;
		#endif
	}
	bool result = initialise_folder( spec.substr( 0, i + 1 ) );
	m_filename = spec.substr( i + 1, spec.size() - i - 1 );
	// TODO - some error handling?
	return result;
}

bool file_specification::simplify( void )
{
	// simplify the path by removing unnecessary . and .. entries - Note that zero-length entries
	// are treated like .
	for( unsigned i = 0; i < m_path.size(); )
	{
		if( m_path[i].empty() || m_path[i].compare( "." ) == 0 )
		{
			// found . or null
			// these both mean do nothing - so simply delete this element
			m_path.erase( m_path.begin() + i );
		}
		else if( m_path[i].compare( ".." ) == 0 )
		{
			// found ..
			if( i == 0 && !m_relative )
			{
				// up from the root does nothing so can be deleted
				m_path.erase( m_path.begin() + i );
				i++;
			}
			else if( i == 0 || m_path[i - 1].compare( ".." ) == 0 )
			{
				// the first element of a relative path or the previous element is .. then keep it
				i++;
			}
			else
			{
				// otherwise delete this element and the previous one
				m_path.erase( m_path.begin() + i );
				m_path.erase( m_path.begin() + i - 1 );
				i--;
			}
		}
			// keep all other elements
		else
		{
			i++;
		}
	}
	// TODO - error checking?
	return true;
}

bool file_specification::make_absolute( const std::string& root )
{
	// test whether already an absolute path in which case there's nothing to do
	if( absolute() )
	{
		return true;
	}
	// now simply call the other version of make_absolute
	file_specification rootspec;
	rootspec.initialise_folder( root );
	return make_absolute( rootspec );
}

bool file_specification::make_absolute( const file_specification& rootspec )
{
	// test whether already an absolute path in which case there's nothing to do
	if( absolute() )
	{
		return true;
	}
	// initialise the result with the root and make the root absolute
	file_specification result = rootspec;
	result.make_absolute();
	// now append this's relative path and filename to the root's absolute path
	for( unsigned i = 0; i < subpath_size(); i++ )
	{
		result.add_subpath( subpath_element( i ) );
	}
	result.set_file( file() );
	// now the result is the absolute path, so transfer it to this
	*this = result;
	// and simplify to get rid of any unwanted .. or . elements
	simplify();
	return true;
}

bool file_specification::make_relative( const std::string& root )
{
	// test whether already an relative path in which case there's nothing to do
	if( relative() )
	{
		return true;
	}
	// now simply call the other version of make_relative
	file_specification rootspec;
	rootspec.initialise_folder( root );
	return make_relative( rootspec );
}

bool file_specification::make_relative( const file_specification& rootspec )
{
	// test whether already an relative path in which case there's nothing to do
	if( relative() )
	{
		return true;
	}
	// initialise the result with the root and make the root absolute
	file_specification absolute_root = rootspec;
	absolute_root.make_absolute();

	// now compare elements of the absolute root with elements of this to find the common path
	// if the drives are different, no conversion can take place and the result must be absolute,
	// else clear the drive
	if( !path_compare( drive(), absolute_root.drive() ) )
	{
		return true;
	}
	set_drive( "" );
	// first remove leading elements that are identical to the corresponding element in root
	unsigned i = 0;
	while( subpath_size() > 0 &&
		   i < absolute_root.subpath_size() &&
		   path_compare( subpath_element( 0 ), absolute_root.subpath_element( i ) ) )
	{
		subpath_erase( 0 );
		i++;
	}
	// now add a .. prefix for every element in root that is different from this
	while( i < absolute_root.subpath_size() )
	{
		m_path.insert( m_path.begin(), ".." );
		i++;
	}
	set_relative();
	return true;
}

std::string file_specification::image( void ) const
{
	std::string result = m_drive;
	if( absolute() )
	{
		result += preferred_separator;
	}
	if( !m_path.empty() )
	{
		for( unsigned i = 0; i < m_path.size(); i++ )
		{
			if( i != 0 )
			{
				result += std::string( 1, preferred_separator );
			}
			result += m_path[i];
		}
	}
	else if( relative() )
	{
		result += '.';
	}
	// add a trailing / to the last directory element
	if( result.empty() || !is_separator( result[result.size() - 1] ) )
	{
		result += preferred_separator;
	}
	if( !m_filename.empty() )
	{
		result += m_filename;
	}
	return result;
}

////////////////////////////////////////////////////////////////////////////////
// classifying functions

// Under both Windows and Unix, the stat function is used for classification

// Under Gnu/Linux, the following classifications are defined
// source: Gnu/Linux man page for stat(2) http://linux.die.net/man/2/stat
//   S_IFMT 	0170000	bitmask for the file type bitfields
//   S_IFSOCK 	0140000	socket (Note this overlaps with S_IFDIR)
//   S_IFLNK 	0120000	symbolic link
//   S_IFREG 	0100000	regular file
//   S_IFBLK 	0060000	block device
//   S_IFDIR 	0040000	directory
//   S_IFCHR 	0020000	character device
//   S_IFIFO 	0010000	FIFO
// There are also some Posix-standard macros:
//   S_ISREG(m)        is it a regular file?
//   S_ISDIR(m)        directory?
//   S_ISCHR(m)        character device?
//   S_ISBLK(m)        block device?
//   S_ISFIFO(m)       FIFO (named pipe)?
//   S_ISLNK(m)        symbolic link? (Not in POSIX.1-1996.)
//   S_ISSOCK(m)       socket? (Not in POSIX.1-1996.)
// Under Windows, the following are defined:
// source: Header file sys/stat.h distributed with Visual Studio 10
//   _S_IFMT  (S_IFMT)   0xF000 file type mask
//   _S_IFREG (S_IFREG)  0x8000 regular
//   _S_IFDIR (S_IFDIR)  0x4000 directory
//   _S_IFCHR (S_IFCHR)  0x2000 character special
//   _S_IFIFO            0x1000 pipe

#ifdef MSWINDOWS
// file type tests are not defined for some reason on Windows despite them providing the stat() function!
#define R_OK 4
#define W_OK 2
// Posix-style macros for Windows
#ifndef S_ISREG
#define S_ISREG(mode)  ((mode & _S_IFMT) == _S_IFREG)
#endif
#ifndef S_ISDIR
#define S_ISDIR(mode)  ((mode & _S_IFMT) == _S_IFDIR)
#endif
#ifndef S_ISCHR
#define S_ISCHR(mode)  ((mode & _S_IFMT) == _S_IFCHR)
#endif
#ifndef S_ISBLK
#define S_ISBLK(mode)  (false)
#endif
#ifndef S_ISFIFO
#define S_ISFIFO(mode) ((mode & _S_IFMT) == _S_IFIFO)
#endif
#ifndef S_ISLNK
#define S_ISLNK(mode)  (false)
#endif
#ifndef S_ISSOCK
#define S_ISSOCK(mode) (false)
#endif
#endif

////////////////////////////////////////////////////////////////////////////////
// file functions

bool file_readable( const std::string& filespec )
{
	// a file is readable if it exists and can be read
	if( !file_exists( filespec ) )
	{
		return false;
	}
	return access( filespec.c_str(), R_OK ) == 0;
}

bool file_writable( const std::string& filespec )
{
	std::error_code ec;

	// a file is writable if it exists as a file and is writable or if
	// it doesn't exist but could be created and would be writable
	bool status = is_present( filespec, ec );
	if( ec )
	{
		throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
	}

	if( status )
	{
		status = is_file( filespec, ec );
		if( ec )
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
		else
		{
			return status;
		}

		return access( filespec.c_str(), W_OK ) == 0;
	}
	std::string dir = folder_part( filespec );
	if( dir.empty() )
	{
		dir = ".";
	}
	return folder_writable( dir );
}

size_t file_size( const std::string& filespec )
{
	struct stat buf;
	if( (stat( filespec.c_str(), &buf ) != 0) )
	{
		return 0;
	}
	return static_cast<size_t>( buf.st_size );
}

bool file_delete( const std::string& filespec )
{
	std::error_code ec;

	const bool isFile = is_file( filespec, ec );

	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	if( !isFile )
	{
		return false;
	}

	if( remove( filespec.c_str() ) == C_API_ERR )
	{
		throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
	}

	return true;
}

bool file_add_permission( const std::string& filespec, const Permission perm )
{
	// a file is readable if it exists and can be read
	if( !file_exists( filespec ) )
	{
		return false;
	}

	mode_t permissions{ };

	if( perm == Permission::UserR )
	{
		permissions = S_IRUSR;
	}
	else if( perm == Permission::UserW )
	{
		permissions = S_IWUSR;
	}
	else if( perm == Permission::UserX )
	{
		permissions = S_IXUSR;
	}
	else if( perm == Permission::UserRWX )
	{
		permissions = S_IRWXU;
	}
	else if( perm == Permission::GroupR )
	{
		permissions = S_IRGRP;
	}
	else if( perm == Permission::GroupW )
	{
		permissions = S_IWGRP;
	}
	else if( perm == Permission::GroupX )
	{
		permissions = S_IXGRP;
	}
	else if( perm == Permission::OtherR )
	{
		permissions = S_IROTH;
	}
	else if( perm == Permission::OtherW )
	{
		permissions = S_IWOTH;
	}
	else if( perm == Permission::OtherX )
	{
		permissions = S_IXOTH;
	}

	return chmod( filespec.c_str(), permissions ) == 0;
}

bool file_rename( const std::string& old_filespec, const std::string& new_filespec )
{
	std::error_code ec;
	const bool isFile = is_file( old_filespec, ec );

	if( ec )
	{
		throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
	}

	if( !isFile )
	{
		return false;
	}

	return rename( old_filespec.c_str(), new_filespec.c_str() ) == 0;
}

bool file_copy( const std::string& old_filespec, const std::string& new_filespec )
{
	std::error_code ec;
	const bool isFile = is_file( old_filespec, ec );

	if( ec )
	{
		throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
	}

	if( !isFile )
	{
		return false;
	}

	// do an exact copy - to do this, use binary mode
	bool result = true;
	FILE* old_file = fopen( old_filespec.c_str(), "rb" );
	FILE* new_file = fopen( new_filespec.c_str(), "wb" );
	if( !old_file )
	{
		result = false;
	}
	else if( !new_file )
	{
		result = false;
	}
	else
	{
		for( int byte = getc( old_file ); byte != EOF; byte = getc( old_file ) )
		{
			putc( byte, new_file );
		}
	}
	if( old_file )
	{
		fflush( old_file );
		fclose( old_file );
	}
	if( new_file )
	{
		fflush( old_file );
		fclose( new_file );
	}

	return result;
}

bool file_move( const std::string& old_filespec, const std::string& new_filespec )
{
	// try to move the file by renaming - if that fails then do a copy and delete the original
	if( file_rename( old_filespec, new_filespec ) )
	{
		return true;
	}
	if( !file_copy( old_filespec, new_filespec ) )
	{
		return false;
	}
	// I'm not sure what to do if the delete fails - is that an error?
	// I've made it an error and then delete the copy so that the original state is recovered
	if( file_delete( old_filespec ) )
	{
		return true;
	}
	file_delete( new_filespec );
	return false;
}

time_t file_created( const std::string& filespec )
{
	struct stat buf;
	if( (stat( filespec.c_str(), &buf ) != 0) )
	{
		return 0;
	}
	return buf.st_ctime;
}

time_t file_modified( const std::string& filespec )
{
	struct stat buf;
	if( (stat( filespec.c_str(), &buf ) != 0) )
	{
		return 0;
	}
	return buf.st_mtime;
}

time_t file_accessed( const std::string& filespec )
{
	struct stat buf;
	if( (stat( filespec.c_str(), &buf ) != 0) )
	{
		return 0;
	}
	return buf.st_atime;
}

std::string create_filespec( const std::string& directory, const std::string& filename )
{
	std::string result = directory;
	// if directory is empty then no directory part will be added
	// add trailing slash if the directory was specified and does not have a trailing slash
	if( !result.empty() && !is_separator( result[result.size() - 1] ) )
	{
		result += preferred_separator;
	}
	// if filename is null or empty, nothing will be added so the path is then a directory path
	result += filename;
	return result;
}

std::string create_filespec( const std::string& directory, const std::string& basename,
							 const std::string& extension )
{
	return create_filespec( directory, create_filename( basename, extension ) );
}

std::string create_filename( const std::string& basename, const std::string& extension )
{
	std::string name = basename;
	// extension is optional - so the dot is also optional
	if( !extension.empty() )
	{
		if( extension[0] != '.' )
		{
			name += '.';
		}
		name += extension;
	}
	return name;
}

////////////////////////////////////////////////////////////////////////////////
// folder functions

std::error_code folder_create( const std::string& directory )
{
	#ifdef MSWINDOWS
	return mkdir(directory.c_str()) == 0;
	#else

	mkdir( directory.c_str(), 0777 );
	std::error_code ec = std::make_error_code( std::errc( errno ) );
	return ec;

	#endif
}

bool folder_exists( const std::string& directory )
{
	std::error_code ec;
	return is_folder( directory, ec );
}

bool folder_readable( const std::string& directory )
{
	// a folder is readable if it exists and has read access
	std::string dir = directory;
	if( dir.empty() )
	{
		dir = ".";
	}
	if( !folder_exists( dir ) )
	{
		return false;
	}
	return access( dir.c_str(), R_OK ) == 0;
}

bool folder_writable( const std::string& directory )
{
	// a folder is writable if it exists and has write access
	std::string dir = directory;
	if( dir.empty() )
	{
		dir = ".";
	}
	if( !folder_exists( dir ) )
	{
		return false;
	}
	return access( dir.c_str(), W_OK ) == 0;
}

bool folder_delete( const std::string& directory, bool recurse )
{
	std::string dir = directory;
	if( dir.empty() )
	{
		dir = ".";
	}
	if( !folder_exists( dir ) )
	{
		return false;
	}

	bool result{ true };

	// depth-first traversal ensures that directory contents are deleted before trying to delete the
	// directory itself
	if( recurse )
	{
		std::vector< std::string > subdirectories = folder_subdirectories( dir );
		for( std::vector< std::string >::size_type d = 0; d < subdirectories.size(); ++d )
		{
			if( !folder_delete( folder_down( dir, subdirectories[d] ), true ) )
			{
				result = false;
			}
		}
		std::vector< std::string > files = folder_files( dir );
		for( std::vector< std::string >::size_type f = 0; f < files.size(); ++f )
		{
			if( !file_delete( create_filespec( dir, files[f] ) ) )
			{
				result = false;
			}
		}
	}
	if( rmdir( dir.c_str() ) != 0 )
	{
		result = false;
	}
	return result;
}

bool folder_copy( const std::string& folder, const std::string& dest )
{
	std::string dir = folder;
	if( dir.empty() )
	{
		dir = ".";
	}
	if( !folder_exists( dir ) )
	{
		return false;
	}

	bool result{ true };

	if( !folder_create( dest ) )
	{
		return false;
	}

	// depth-first traversal ensures that directory contents are deleted before trying to delete the
	// directory itself
	std::vector< std::string > subdirectories = folder_subdirectories( dir );

	for( std::vector< std::string >::size_type d = 0; d < subdirectories.size(); ++d )
	{
		folder_copy( folder_down( dir, subdirectories[d] ),
					 folder_to_path( dest, subdirectories[d] ) );
	}

	std::vector< std::string > files = folder_files( dir );
	for( std::vector< std::string >::size_type f = 0; f < files.size(); ++f )
	{
		std::string src = create_filespec( dir, files[f] );
		std::string dst = create_filespec( dest, files[f] );

		if( !file_copy( src, dst ) )
		{
			result = false;
		}
	}
	return result;
}

bool folder_rename( const std::string& old_directory, const std::string& new_directory )
{
	if( !folder_exists( old_directory ) )
	{
		return false;
	}
	return rename( old_directory.c_str(), new_directory.c_str() ) == 0;
}

bool folder_empty( const std::string& directory )
{
	std::string dir = directory.empty() ? std::string( "." ) : directory;
	bool result = true;
	#ifdef MSWINDOWS
	std::string wildcard = create_filespec(dir, "*.*");
	intptr_t handle = -1;
	_finddata_t fileinfo;
	for (bool OK = (handle = _findfirst((char*)wildcard.c_str(), &fileinfo)) != -1; OK;
		OK = (_findnext(handle, &fileinfo)==0))
	{
	  std::string strentry = fileinfo.name;
	  if (strentry.compare(".")!=0 && strentry.compare("..")!=0)
	  {
		result = false;
		break;
	  }
	}
	_findclose(handle);
	#else
	DIR* d = opendir( dir.c_str() );
	if( d )
	{
		for( dirent* entry = readdir( d ); entry; entry = readdir( d ) )
		{
			std::string strentry = entry->d_name;
			if( strentry.compare( "." ) != 0 && strentry.compare( ".." ) != 0 )
			{
				result = false;
				break;
			}
		}
		closedir( d );
	}
	#endif
	return result;
}

bool folder_set_current( const std::string& folder )
{
	if( !folder_exists( folder ) )
	{
		return false;
	}
	#ifdef MSWINDOWS
	// Windose implementation - this returns non-zero for success
	return (SetCurrentDirectoryA(folder.c_str()) != 0);
	#else
	// Unix implementation - this returns zero for success
	return (chdir( folder.c_str() ) == 0);
	#endif
}

std::string folder_current( void )
{
	return ".";
}

std::string folder_current_full( void )
{
	// It's not clear from the documentation whether the buffer for a path should be one byte longer
	// than the maximum path length to allow for the null termination, so I have made it so anyway
	#ifdef MSWINDOWS
	char abspath [MAX_PATH+1];
	return std::string(_fullpath(abspath, ".", MAX_PATH+1));
	#else
	char pathname[MAXPATHLEN + 1];
	char* result = getcwd( pathname, MAXPATHLEN + 1 );
	if( !result )
	{
		// should really report the error from errno
		return std::string();
	}
	return std::string( result );
	#endif
}

std::string folder_down( const std::string& directory, const std::string& subdirectory )
{
	file_specification spec;
	spec.initialise_folder( directory );
	spec.add_subpath( subdirectory );
	return spec.image();
}

std::string folder_up( const std::string& directory, unsigned levels )
{
	file_specification spec;
	spec.initialise_folder( directory );
	for( unsigned i = 0; i < levels; i++ )
	{
		spec.add_subpath( ".." );
	}
	spec.simplify();
	return spec.image();
}

std::vector< std::string > folder_subdirectories( const std::string& directory )
{
	return folder_wildcard( directory, "*", true, false );
}

std::vector< std::string > folder_files( const std::string& directory )
{
	return folder_wildcard( directory, "*", false, true );
}

std::vector< std::string > folder_all( const std::string& directory )
{
	return folder_wildcard( directory, "*", true, true );
}

std::vector< std::string > folder_wildcard( const std::string& directory, const std::string& wild,
											bool subdirs, bool files )
{
	std::string dir = directory.empty() ? std::string( "." ) : directory;
	std::vector< std::string > results;

	#ifdef MSWINDOWS
	std::string wildcard = create_filespec(dir, wild);
	intptr_t handle = -1;
	_finddata_t fileinfo;
	for (bool OK = (handle = _findfirst((char*)wildcard.c_str(), &fileinfo)) != -1; OK;
	OK = (_findnext(handle, &fileinfo)==0))
	{
	  std::string strentry = fileinfo.name;
	  if (strentry.compare(".")!=0 && strentry.compare("..")!=0)
		if ((subdirs && (fileinfo.attrib & _A_SUBDIR)) || (files && !(fileinfo.attrib & _A_SUBDIR)))
		  results.push_back(strentry);
	}
	_findclose(handle);
	#else
	DIR* d = opendir( dir.c_str() );
	if( d )
	{
		for( dirent* entry = readdir( d ); entry; entry = readdir( d ) )
		{
			std::string strentry = entry->d_name;
			if( strentry.compare( "." ) != 0 && strentry.compare( ".." ) != 0 )
			{
				std::error_code ec;
				std::string subpath = create_filespec( dir, strentry );

				if( ((subdirs && is_folder( subpath, ec )) || (files && is_file( subpath, ec ))) &&
					(wildcard( wild, strentry )) )
				{
					results.push_back( strentry );
				}
			}
		}
		closedir( d );
	}
	#endif
	return results;
}

std::string folder_home( void )
{
	if( getenv( "HOME" ) )
	{
		return std::string( getenv( "HOME" ) );
	}
	#ifdef MSWINDOWS
	if (getenv("HOMEDRIVE") || getenv("HOMEPATH"))
	  return std::string(getenv("HOMEDRIVE")) + std::string(getenv("HOMEPATH"));
	return "C:\\";
	#else
	if( getenv( "USER" ) )
	{
		return folder_down( "/home", std::string( getenv( "USER" ) ) );
	}
	if( getenv( "USERNAME" ) )
	{
		return folder_down( "/home", std::string( getenv( "USERNAME" ) ) );
	}
	return "";
	#endif
}

////////////////////////////////////////////////////////////////////////////////
// path functions convert between full and relative paths

bool is_full_path( const std::string& path )
{
	file_specification spec;
	spec.initialise_folder( path.empty() ? std::string( "." ) : path );
	return spec.absolute();
}

bool is_relative_path( const std::string& path )
{
	file_specification spec;
	spec.initialise_folder( path.empty() ? std::string( "." ) : path );
	return spec.relative();
}

static std::string full_path( const std::string& root, const std::string& path )
{
	// convert path to a full path using root as the start point for relative paths
	// decompose the path and test whether it is already an absolute path, in which case just return it
	file_specification spec;
	spec.initialise_folder( path.empty() ? std::string( "." ) : path );
	if( spec.absolute() )
	{
		return spec.image();
	}
	// okay, so the path is relative after all, so we need to combine it with the root path
	// decompose the root path and check whether it is relative
	file_specification rootspec;
	rootspec.initialise_folder( root.empty() ? std::string( "." ) : root );
	if( rootspec.relative() )
	{
		rootspec.make_absolute();
	}
	// Now do the conversion of the path relative to the root
	spec.make_absolute( rootspec );
	return spec.image();
}

static std::string relative_path( const std::string& root, const std::string& path )
{
	// convert path to a relative path, using the root path as its starting point
	// first convert both paths to full paths relative to CWD
	file_specification rootspec;
	rootspec.initialise_folder( root.empty() ? std::string( "." ) : root );
	if( rootspec.relative() )
	{
		rootspec.make_absolute();
	}
	file_specification spec;
	spec.initialise_folder( path.empty() ? std::string( "." ) : path );
	if( spec.relative() )
	{
		spec.make_absolute();
	}
	// now make path spec relative to the root spec
	spec.make_relative( rootspec );
	return spec.image();
}

std::string folder_to_path( const std::string& path, const std::string& directory )
{
	return full_path( path, directory );
}

std::string filespec_to_path( const std::string& path, const std::string& spec )
{
	return create_filespec( folder_to_path( path, folder_part( spec ) ), filename_part( spec ) );
}

std::string folder_to_path( const std::string& folder )
{
	return folder_to_path( folder_current(), folder );
}

std::string filespec_to_path( const std::string& filespec )
{
	return filespec_to_path( folder_current(), filespec );
}

std::string folder_to_relative_path( const std::string& root, const std::string& folder )
{
	return relative_path( root, folder );
}

std::string filespec_to_relative_path( const std::string& root, const std::string& spec )
{
	return create_filespec( folder_to_relative_path( root, folder_part( spec ) ),
							filename_part( spec ) );
}

std::string folder_to_relative_path( const std::string& folder )
{
	return folder_to_relative_path( folder_current(), folder );
}

std::string filespec_to_relative_path( const std::string& filespec )
{
	return filespec_to_relative_path( folder_current(), filespec );
}

std::string folder_append_separator( const std::string& folder )
{
	std::string result = folder;
	if( result.empty() || !is_separator( result[result.size() - 1] ) )
	{
		result += preferred_separator;
	}
	return result;
}

////////////////////////////////////////////////////////////////////////////////

std::string basename_part( const std::string& spec )
{
	std::string fname = filename_part( spec );
	// scan back through filename until a '.' is found and remove suffix
	// the whole filename is the basename if there is no '.'
	std::string::size_type i = fname.find_first_of( '.' );
	// observe Unix convention that a dot at the start of a filename is part of the basename, not
	// the extension
	if( i != 0 && i != std::string::npos )
	{
		fname.erase( i, fname.size() - i );
	}
	return fname;
}

std::string filename_part( const std::string& spec )
{
	// scan back through filename until a preferred_separator is found and remove prefix;
	// if there is no preferred_separator then remove nothing, i.e. the whole filespec is filename
	size_t i = spec.size();
	while( i-- )
	{
		if( is_separator( spec[i] ) )
		{
			return spec.substr( i + 1, spec.size() - i - 1 );
		}
	}
	return spec;
}

std::string extension_part( const std::string& spec )
{
	std::string fname = filename_part( spec );

	// scan back through filename until a '.' is found and remove prefix;
	std::string::size_type i = fname.find_first_of( '.' );

	// observe Unix convention that a dot at the start of a filename is part of the name, not the
	// extension;
	if( i != 0 && i != std::string::npos )
	{
		fname.erase( 0, i + 1 );
	}
	else
	{
		fname.erase();
	}
	return fname;
}

std::string folder_part( const std::string& spec )
{
	// scan back through filename until a separator is found and remove prefix
	// if there is no separator, remove the whole
	size_t i = spec.size();
	while( i-- )
	{
		if( is_separator( spec[i] ) )
		{
			return spec.substr( 0, i );
		}
	}
	return std::string();
}

std::vector< std::string > filespec_elements( const std::string& filespec )
{
	file_specification spec;
	spec.initialise_file( filespec );
	std::vector< std::string > result = spec.path();
	if( !spec.drive().empty() )
	{
		result.insert( result.begin(), spec.drive() );
	}
	if( !spec.file().empty() )
	{
		result.push_back( spec.file() );
	}
	return result;
}

std::vector< std::string > folder_elements( const std::string& folder )
{
	file_specification spec;
	spec.initialise_folder( folder );
	std::vector< std::string > result = spec.path();
	if( !spec.drive().empty() )
	{
		result.insert( result.begin(), spec.drive() );
	}
	return result;
}

////////////////////////////////////////////////////////////////////////////////
// mimic the command lookup used by the shell

// Windows looks at the following locations:
// 1) application root
// 2) current directory
// 3) 32-bit system directory
// 4) 16-bit system directory
// 5) windows system directory
// 6) %path%
// currently only (2) and (6) has been implemented although many system folders are on the path
// anyway also implement the implied .exe extension on commands with no path (see CreateProcess
// documentation)
// TODO - PATHEXT handling to find non-exe executables

std::string path_lookup( const std::string& command )
{
	std::string path = std::string( "." ) + PATH_SPLITTER + getenv( "PATH" );
	return lookup( command, path );
}

std::string lookup( const std::string& command, const std::string& path,
					const std::string& splitter )
{
	// first check whether the command is already a path and check whether it exists
	if( !folder_part( command ).empty() )
	{
		if( file_exists( command ) )
		{
			return command;
		}
	}
	else
	{
		// command is just a name - so do path lookup
		// split the path into its elements
		std::vector< std::string > paths;
		if( !path.empty() )
		{
			for( std::string::size_type offset = 0; ; )
			{
				std::string::size_type found = path.find( splitter, offset );
				if( found != std::string::npos )
				{
					paths.push_back( path.substr( offset, found - offset ) );
					offset = found + splitter.size();
				}
				else
				{
					paths.push_back( path.substr( offset, path.size() - offset ) );
					break;
				}
			}
		}
		// now lookup each path to see if it its the matching one
		for( unsigned i = 0; i < paths.size(); i++ )
		{
			std::string spec = create_filespec( paths[i], command );
			if( file_exists( spec ) )
			{
				return spec;
			}
		}
	}
	#ifdef MSWINDOWS
	// if there is no extension, try recursing on each possible extension
	// TODO iterate through PATHEXT
	if (extension_part(command).empty())
	  return lookup(create_filespec(folder_part(command), basename_part(command), "exe"), path, splitter);
	#endif
	// if path lookup failed, return empty string to indicate error
	return std::string();
}

////////////////////////////////////////////////////////////////////////////////

std::string install_path( const std::string& argv0 )
{
	std::string bin_directory = folder_part( argv0 );
	if( bin_directory.empty() )
	{
		// do path lookup to find the executable path
		bin_directory = folder_part( path_lookup( argv0 ) );
	}
	return bin_directory;
}

////////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------------------
//
bool
is_drive_mounted( const std::string& devPath, std::string& mountPoint )
{
	FILE* mtab{ };
	struct mntent* part{ };

	bool mounted{ };

	if( (mtab = setmntent( "/etc/mtab", "r" )) != NULL )
	{
		while( (part = getmntent( mtab )) != NULL )
		{
			if( (part->mnt_fsname != NULL) && (devPath.compare( part->mnt_fsname ) == 0) )
			{
				mountPoint = part->mnt_dir;
				mounted = true;
			}
		}

		endmntent( mtab );
	}

	return mounted;
}

//--------------------------------------------------------------------------------------------------
//
bool
mount_drive( const std::string& devPath, const std::string& dirPath, std::error_code& ec,
			 const bool readOnly )
{
	if( !cl::filesystem::folder_exists( dirPath ) )
	{
		ec = cl::filesystem::folder_create( dirPath );
		if( ec )
		{
			return false;
		}
	}

	uint32_t options{ MS_MGC_VAL | MS_NOSUID };
	if( readOnly )
	{
		options |= MS_RDONLY;
	}

	if( mount( devPath.c_str(), dirPath.c_str(), "vfat", options, "" ) == C_API_ERR )
	{
		if( mount( devPath.c_str(), dirPath.c_str(), "ntfs", options, "" ) == C_API_ERR )
		{
			ec = std::make_error_code( std::errc( errno ) );
			return false;
		}
	}
	return true;
}

//--------------------------------------------------------------------------------------------------
//
std::error_code
unmount_drive( const std::string& devPath )
{
	std::error_code ec{ };
	if( umount( devPath.c_str() ) == C_API_ERR )
	{
		ec = std::make_error_code( std::errc( errno ) );
	}
	return ec;
}

//--------------------------------------------------------------------------------------------------
//
uint64_t
get_free_space( const std::string& mntPath )
{
	struct statvfs info;

	if( statvfs( mntPath.c_str(), &info ) != 0 )
	{
		throw cl::SystemError( errno, std::generic_category(), CL_ORIGIN );
	}

	return static_cast< uint64_t >( info.f_bavail * info.f_bsize );
}

} // end namespace stlplus

}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::is_present( const std::string& thing, std::error_code& ec )
{
	// strip off any trailing separator because that will cause the stat function to fail
	std::string path = remove_trailing_slash( thing );

	// now test if this thing exists using the built-in stat function
	struct stat buf;
	stat( path, buf, ec );
	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	return true;
}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::is_folder( const std::string& thing, std::error_code& ec )
{
	// strip off any trailing separator because that will cause the stat function to fail
	std::string path = remove_trailing_slash( thing );

	// now test if this thing exists using the built-in stat function and if so, is it a folder
	struct stat buf;
	stat( path, buf, ec );
	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	// If the object is present, see if it is a directory this is the Posix-approved way of testing
	return S_ISDIR( buf.st_mode );
}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::is_file( const std::string& thing, std::error_code& ec )
{
	// strip off any trailing separator because that will cause the stat function to fail
	std::string path = remove_trailing_slash( thing );

	// now test if this thing exists using the built-in stat function and if so, is it a file
	struct stat buf;
	stat( path, buf, ec );
	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	// If the object is present, see if it is a file or file-like object
	// Note that devices are neither folders nor files
	// this is the Posix-approved way of testing
	return S_ISREG( buf.st_mode ) || S_ISLNK( buf.st_mode ) || S_ISSOCK( buf.st_mode ) ||
		   S_ISBLK( buf.st_mode ) || S_ISFIFO( buf.st_mode );
}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::is_regular_file( const std::string& thing, std::error_code& ec )
{
	// strip off any trailing separator because that will cause the stat function to fail
	std::string path = remove_trailing_slash( thing );

	// now test if this thing exists using the built-in stat function and if so, is it a file
	struct stat buf;
	lstat( path, buf, ec );
	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	// If the object is present, see if it is a file or file-like object. Note that devices are
	// neither folders nor files this is the Posix-approved way of testing
	return S_ISREG( buf.st_mode );
}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::is_fifo_file( const std::string& thing, std::error_code& ec )
{
	// strip off any trailing separator because that will cause the stat function to fail
	std::string path = remove_trailing_slash( thing );

	// now test if this thing exists using the built-in stat function and if so, is it a file
	struct stat buf;
	lstat( path, buf, ec );
	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	// If the object is present, see if it is a file or file-like object. Note that devices are
	// neither folders nor files this is the Posix-approved way of testing
	return S_ISFIFO( buf.st_mode );
}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::is_socket_file( const std::string& thing, std::error_code& ec )
{
	// strip off any trailing separator because that will cause the stat function to fail
	std::string path = remove_trailing_slash( thing );

	// now test if this thing exists using the built-in stat function and if so, is it a file
	struct stat buf;
	stat( path, buf, ec );
	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	// If the object is present, see if it is a file or file-like object. Note that devices are
	// neither folders nor files this is the Posix-approved way of testing
	return S_ISSOCK( buf.st_mode );
}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::is_symlink_file( const std::string& thing, std::error_code& ec )
{
	// strip off any trailing separator because that will cause the stat function to fail
	std::string path = remove_trailing_slash( thing );

	// now test if this thing exists using the built-in stat function and if so, is it a file
	struct stat buf;
	lstat( path, buf, ec );
	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	// If the object is present, see if it is a file or file-like object Note that devices are
	// neither folders nor files this is the Posix-approved way of testing
	return S_ISLNK( buf.st_mode );
}

//--------------------------------------------------------------------------------------------------
//
void
cl::filesystem::create_file( const std::string& filepath, std::error_code& ec )
{
	int32_t fd = open( filepath.c_str(), O_RDWR | O_CREAT, 666 );
	if( fd == -1 )
	{
		ec.assign( errno, std::generic_category() );
	}
}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::file_exists( const std::string& filespec )
{
	std::error_code ec;
	const bool isFile = is_file( filespec, ec );

	if( ec )
	{
		if( ec == std::make_error_code( std::errc::no_such_file_or_directory ) )
		{
			return false;
		}
		else
		{
			throw cl::SystemError( ec.value(), ec.category(), CL_ORIGIN );
		}
	}

	return isFile;
}

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//------------------------------------------ EnumVisitor -------------------------------------------
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
cl::filesystem::EnumVisitor::EnumVisitor()
	: isDirectory_{ }
	, wasLink_{ }
{ }

//--------------------------------------------------------------------------------------------------
//
cl::filesystem::EnumVisitor::~EnumVisitor()
{ }

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::EnumVisitor::is_directory() const
{
	return isDirectory_;
}

//--------------------------------------------------------------------------------------------------
//
bool
cl::filesystem::EnumVisitor::was_link() const
{
	return wasLink_;
}

//--------------------------------------------------------------------------------------------------
//
void
cl::filesystem::EnumVisitor::set_directory( const bool isDirectory )
{
	isDirectory_ = isDirectory;
}

//--------------------------------------------------------------------------------------------------
//
void
cl::filesystem::EnumVisitor::set_was_link( const bool wasLink )
{
	wasLink_ = wasLink;
}
//
////--------------------------------------------------------------------------------------------------
////--------------------------------------------------------------------------------------------------
////------------------------------------------- UNIT TESTS -------------------------------------------
////--------------------------------------------------------------------------------------------------
////--------------------------------------------------------------------------------------------------
//
//#ifdef DEBUG
//#include "Common/Print.hpp"
//
////==================================================================================================
//// U N I T   T E S T S   C O D E   S E C T I O N
//
//TC_DEFINE_UNIT_TEST( CLFileSystemUT )
//	{
//		std::string testFolder{ "test_folder" };
//		std::string testFile{ "test_file" };
//		std::error_code ec;
//
//		TC_TEST_DIE( !cl::filesystem::is_present( testFolder, ec ) &&
//					 ec == std::make_error_code( std::errc::no_such_file_or_directory ) );
//		ec.clear();
//
//		TC_TEST_DIE( !cl::filesystem::is_folder( testFolder, ec ) &&
//					 ec == std::make_error_code( std::errc::no_such_file_or_directory ) );
//		ec.clear();
//
//		TC_TEST_DIE( !cl::filesystem::is_file( testFile, ec ) &&
//					 ec == std::make_error_code( std::errc::no_such_file_or_directory ) );
//		ec.clear();
//
//		TC_TEST_DIE( !cl::filesystem::is_regular_file( testFile, ec ) &&
//					 ec == std::make_error_code( std::errc::no_such_file_or_directory ) );
//		ec.clear();
//
//		TC_TEST_DIE( !cl::filesystem::is_fifo_file( testFile, ec ) &&
//					 ec == std::make_error_code( std::errc::no_such_file_or_directory ) );
//		ec.clear();
//
//		TC_TEST_DIE( !cl::filesystem::is_symlink_file( testFile, ec ) &&
//					 ec == std::make_error_code( std::errc::no_such_file_or_directory ) );
//		ec.clear();
//
//		TC_TEST_DIE( !cl::filesystem::is_socket_file( testFile, ec ) &&
//					 ec == std::make_error_code( std::errc::no_such_file_or_directory ) );
//		ec.clear();
//
//		TC_TEST_DIE( !cl::filesystem::file_exists( testFile ) );
//		ec.clear();
//
//		cl::filesystem::create_file( testFile, ec );
//		TC_TEST_DIE( !ec );
//
//		TC_TEST_DIE( cl::filesystem::file_exists( testFile ) );
//		ec.clear();
//
//		TC_TEST_DIE( cl::filesystem::is_regular_file( testFile, ec ) && !ec );
//		ec.clear();
//
//		TC_TEST_DIE( cl::filesystem::file_delete( testFile ) );
//
//		return true;
//	}
//TC_END_UNIT_TEST( CLFileSystemUT )
//
//#endif
