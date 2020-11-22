#ifndef TWcount_h
#define TWcount_h

#include "../base/defines.h"
#include <iostream>
#include <sstream>
#include <iomanip>

#include "conversion.h"



struct count_type {
	std::string ToString() {
		std::stringstream s;
		s << std::setw(10) << f << std::setw(10) << df << std::setw(10) << dfp;

		return s.str();

	}
	

	count_type() {
		f=0;
		df=0;
		dfp=0;
	}

	int f;
	int df;
	int dfp;
};

/** Zählen von TransWORHP Aufrufen Zielfkt, Nebenbed, etc. */
struct TWcount_calls {
	count_type ode;
	count_type rand;
	count_type obj;
	count_type neben;
	count_type integrate;
};

#endif
