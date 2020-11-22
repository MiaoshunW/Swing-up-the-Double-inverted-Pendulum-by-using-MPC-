//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef vektor_h
#define vektor_h

#include "defines.h"

#ifdef WIN32
#include "windows.h"
#endif
#include <iostream>

#ifdef WIN32
#ifndef MINGW
#define DllExport __declspec( dllexport )
#pragma warning (disable : 4251)
#else
#define DllExport
#endif
#else 
#define DllExport
#endif

/** @ingroup base
 *  @brief Arithmetics for the IR^3, IN^3, ...
 *
 */
template <typename T>
class DllExport Vektor {
public:
	/** Constructor. */
	Vektor() : x(0), y(0), z(0) {}
	/**
	 * Constructor.
	 * 
	 * @param v[] 
	 * @return 
	 */
	Vektor(T v[3]) : x(v[0]), y(v[1]), z(v[2]) {}
	/**
	 * Constructor.
	 * 
	 * @param x1 
	 * @param y1 
	 * @param z1 
	 * @return 
	 */
	Vektor(T x1, T y1, T z1) : x(x1), y(y1), z(z1) {}

	/**
	 * 
	 * @param other 
	 * @return 
	 */
	Vektor<T> operator+(const Vektor<T> &other) const;
	/**
	 * 
	 * @param other 
	 * @return 
	 */
	Vektor<T> operator-(const Vektor<T> &other) const;
	/**
	 * 
	 * @param other 
	 * @return 
	 */
	Vektor<T> operator*(T other) const;
	/**
	 * 
	 * @param other 
	 * @return 
	 */
	Vektor<T> operator/(T other) const;

	/**
	 * 
	 * @param other 
	 * @return 
	 */
	Vektor<T>& operator+=(const Vektor<T> &other);
	/**
	 * 
	 * @param other 
	 * @return 
	 */
	Vektor<T>& operator-=(const Vektor<T> &other);
	/**
	 * 
	 * @param other 
	 * @return 
	 */
	Vektor<T>& operator*=(T other);
	/**
	 * 
	 * @param other 
	 * @return 
	 */
	Vektor<T>& operator/=(T other);

	/**
	 * Cross product between to vectors.
	 * @param other 
	 * @return 
	 */
	Vektor<T> cross(const Vektor<T> &other) const;
	
	/**
	 * Skalar product between to vectors.
	 * @param other 
	 * @return 
	 */
	T operator*(const Vektor<T> &other) const;

	/**
	 * Inner product between to vectors.
	 * @param other 
	 * @return 
	 */
	Vektor<T> operator|(const Vektor<T> &other) const;


	/**
	 * Get length of vector
	 * @return 
	 */
	T Length() const;
	/**
	 * Rotate around x axis.
	 * @param angle 
	 * @return 
	 */
	Vektor rotX(T angle) const;
	/**
	 * Rotate around y axis. 
	 * @param angle 
	 * @return 
	 */
	Vektor rotY(T angle) const;
	/**
	 * Rotate around z axis.
	 * @param angle 
	 * @return 
	 */
	Vektor rotZ(T angle) const;
	/**
	 * Normalize vector.
	 */
	void ReduceToUnit();

	/**
	 * Get coordinates as array .
	 * @return
	 */
	T *data() {
		return &x;
	}

	T X() const {
		return x;
	}
	T Y() const {
		return y;
	}
	T Z() const {
		return z;
	}

	T& X() {
		return x;
	}
	T& Y() {
		return y;
	}
	T& Z() {
		return z;
	}
	/*void X(T a) {
		x=a;
	}
	void Y(T a) {
		y=a;
	}
	void Z(T a) {
		z=a;
	}*/

	friend std::ostream &operator<<(std::ostream &os, const Vektor<T> &v) {
		os << "[" << v.x << "," << v.y << "," << v.z << "]" ;
		return os;
	}
	friend std::istream &operator>>(std::istream &is, Vektor<T> &v) {
		is >> v.x >> v.y >> v.z;
		return is;
	}


	Vektor<T> rotate(const Vektor<T> &axis, T angle) const;

private:
	T x,y,z;

};

/** @ingroup base
 * Calculate normal vector.
 * Returns vector perpendicular to u, v and w.
 * @param u 
 * @param v 
 * @param w 
 * @param out 
 */ 
template <class T>
void calcNormal(const Vektor<T> &u, const Vektor<T> &v, const Vektor<T> &w, Vektor<T> &out);

// template <class T>
// Vektor<T> abs(const Vektor<T> &u);
// 
// template <class T>
// Vektor<T> log(const Vektor<T> &u);



#endif



