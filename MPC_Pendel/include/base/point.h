//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef point_h
#define point_h
#include "defines.h"
#include <iostream>
#include <cmath>

/** @defgroup glbase Basics for OpenGL
 *  @brief ...
 */


/** @ingroup glbase
 *  @brief Point with (x/y) integer coordinates.
 * 
 */
template <typename T>
struct Point {

	Point() : x(0),y(0) {}
	Point(T x_, T y_) : x(x_),y(y_) {}
	Point(const Point &other) : x(other.x),y(other.y) {}

	Point &operator=(const Point &other) {
		x = other.x;
		y = other.y;
		return *this;
	}
	Point operator+(const Point &other) const {
		Point p;
		p.x = x+other.x;
		p.y = y+other.y;
		return p;
	}
	Point operator*(T v) const {
		Point p;
		p.x = x*v;
		p.y = y*v;
		return p;
	}
	Point operator-(const Point &other) const {
		Point p;
		p.x = x-other.x;
		p.y = y-other.y;
		return p;
	}
	Point ReduceToUnit() const {
		T len = (T)std::sqrt((double)(x*x + y*y));
		
		if (len>1e-6) return Point(x/len,y/len);		
		return Point(0,0);
	}
	Point rotleft() const {
		return Point(-y,x);
	}
	Point rotright() const {
		return Point(y,-x);
	}
	
	bool isin(const Point &c1, const Point &c2) const;

	T *data() {
		return &x;
	}

	T x,y;


	friend std::ostream &operator<<(std::ostream &os, const Point<T> &p) {

		os << "[" << p.x<< ";"<< p.y<<"]";
		return os;
	}
	
	bool operator<(const Point<T> &other) const;

};



#endif // point
