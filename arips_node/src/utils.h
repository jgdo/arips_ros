/* 
 * File:   utils.h
 * Author: mork
 *
 * Created on February 5, 2011, 12:42 PM
 */

#ifndef UTILS_H
#define	UTILS_H

/**
 * Limits a value to lower and upper boundings.
 *
 * @param val value to limit
 * @param min upper bounding
 * @param max lower bounding
 * @return value between min and max
 */
template<typename T>
inline T Limit(T val, T min, T max)
{
	if(val < min)
		return min;
	else if(val > max)
		return max;
	return val;
}


#endif	/* UTILS_H */

