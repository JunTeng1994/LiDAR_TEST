#include "angles.h"
#define M_PI 3.14159265358979323846
namespace angles {

	double from_hours(double angle)
	{
		return angle*M_PI / 12.;
	}

	double to_hours(double angle)
	{
		return angle*12. / M_PI;
	}

	double from_degrees(double angle)
	{
		return angle*M_PI / 180.;
	}

	double to_degrees(double angle)
	{
		return angle*180. / M_PI;
	}

	double from_arcmin(double angle)
	{
		return (angle / 60.)*M_PI / 180.;
	}

	double to_arcmin(double angle)
	{
		return angle*(180. / M_PI)*60.;
	}

	double from_arcsec(double angle)
	{
		return (angle / 3600.)*M_PI / 180.;
	}

	double to_arcsec(double angle)
	{
		return angle*(180. / M_PI)*3600.;
	}

}