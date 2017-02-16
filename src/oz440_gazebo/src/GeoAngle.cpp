#include "GeoAngle.hpp"

#include <cmath>

// #################################################
//
GeoAngle GeoAngle::from_double( double angle_in_degrees )
{
	double angleInDegrees = angle_in_degrees;
	GeoAngle result;

	//ensure the value will fall within the primary range [-180.0..+180.0]
	while ( angleInDegrees < -180.0)
	{
		angleInDegrees += 360.0;
	}

	while (angleInDegrees > 180.0)
	{
		angleInDegrees -= 360.0;
	}

	//switch the value to positive
	result.IsNegative = angleInDegrees < 0;
	angleInDegrees = std::abs( angleInDegrees );

	//gets the degree
	result.Degrees = (int)std::floor( angleInDegrees );
	double delta = angleInDegrees - result.Degrees;

	//gets minutes and seconds
	int seconds = (int)std::floor( 3600.0 * delta );
	result.Seconds = seconds % 60;
	result.Minutes = (int)std::floor( seconds / 60.0 );
	delta = delta * 3600.0 - seconds;

	//gets fractions
	result.Milliseconds = (int)( 1000.0 * delta );

	return result;
}

// #################################################
//
std::string GeoAngle::to_string( bool is_ns )
{
	char buffer[ 127 ];

	if( is_ns )
	{
		sprintf( buffer, "%02d%02d.%02d%03d,%s", Degrees, Minutes, Seconds, Milliseconds, IsNegative ? "S" : "N" );
	}
	else
	{
		sprintf( buffer, "%03d%02d.%02d%03d,%s", Degrees, Minutes, Seconds, Milliseconds, IsNegative ? "W" : "E" );
	}

	return std::string( buffer );
}
