#ifndef GEO_ANGLE_HPP
#define GEO_ANGLE_HPP

#include <cstring>
#include <iostream>

class GeoAngle
{
	public:

	GeoAngle( ){ };
	~GeoAngle( ){ };

	static GeoAngle from_double( double angle_in_degrees );

	std::string to_string( bool is_ns );

	public:

	bool IsNegative;
	int Degrees;
	int Minutes;
	int Seconds;
	int Milliseconds;
};

#endif //GEO_ANGLE_HPP