/* 
 * File:   angle.h
 * Author: jgdo
 *
 * Created on 4. Februar 2011, 18:50
 */

#ifndef ANGLE_H
#define	ANGLE_H

#include <cmath>

template<class T>
class Degree;

template<class T>
struct Radian {
	T value;

	inline Radian() {
		value = 0;
	}

	inline Radian(T rad) :
	value(rad) {
	}

	/**
	 * @return Angle in radian.
	 */
	inline operator T() const {
		return value;
	}

	void Normalize() {
		value -= (floor(value / (2 * M_PI)) * 2 * M_PI);
		if(value > M_PI)
			value -= 2*M_PI;
	}

	inline operator Degree<T > () const {
		return Degree<T > (value * 180.0 / M_PI);
	}

	inline Radian<T> operator+(Radian<T> r) {
		return value + r;
	}
	
	inline Radian<T> operator-(Radian<T> r) {
			return Radian<T>(value - r);
		}

	inline void operator +=(T val) {
		value += val;
		Normalize();
	}
};

template<class T>
struct Degree {
	T value;

	inline Degree() {
		value = 0;
	}

	inline Degree(T deg) :
	value(deg) {
	}

	/**
	 * @return Angle in radian.
	 */
	inline operator T() const {
		return Radian<T > (*this);
	}

	void Normalize() {
		value -= (floor(value / 360) * 360);
	}

	inline operator Radian<T > () const {
		return Radian<T > (value * M_PI / 180.0);
	}
};

typedef Radian<double> Rad;
typedef Radian<float> Radf;
typedef Degree<double> Deg;
typedef Degree<float> Degf;

#endif	/* ANGLE_H */

