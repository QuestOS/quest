/*                    The Quest Operating System
 *  Copyright (C) 2005-2012  Richard West, Boston University
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All
 * rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#ifndef _MATH_H_
#define _MATH_H_

#define M_PI 3.14159265358979323846
#define M_E 2.7182818284590452354

extern double q_fabs(double t);
extern float q_fabsf(float t);

extern double q_round(double x);
extern int q_lroundf(float x);

/* __ieee754_pow(x,y) return x**y
 *
 *                    n
 * Method:  Let x =  2   * (1+f)
 *      1. Compute and return log2(x) in two pieces:
 *              log2(x) = w1 + w2,
 *         where w1 has 53-24 = 29 bit trailing zeros.
 *      2. Perform y*log2(x) = n+y' by simulating muti-precision
 *         arithmetic, where |y'|<=0.5.
 *      3. Return x**y = 2**n*exp(y'*log2)
 *
 * Special cases:
 *      1.  +-1 ** anything  is 1.0
 *      2.  +-1 ** +-INF     is 1.0
 *      3.  (anything) ** 0  is 1
 *      4.  (anything) ** 1  is itself
 *      5.  (anything) ** NAN is NAN
 *      6.  NAN ** (anything except 0) is NAN
 *      7.  +-(|x| > 1) **  +INF is +INF
 *      8.  +-(|x| > 1) **  -INF is +0
 *      9.  +-(|x| < 1) **  +INF is +0
 *      10  +-(|x| < 1) **  -INF is +INF
 *      11. +0 ** (+anything except 0, NAN)               is +0
 *      12. -0 ** (+anything except 0, NAN, odd integer)  is +0
 *      13. +0 ** (-anything except 0, NAN)               is +INF
 *      14. -0 ** (-anything except 0, NAN, odd integer)  is +INF
 *      15. -0 ** (odd integer) = -( +0 ** (odd integer) )
 *      16. +INF ** (+anything except 0,NAN) is +INF
 *      17. +INF ** (-anything except 0,NAN) is +0
 *      18. -INF ** (anything)  = -0 ** (-anything)
 *      19. (-anything) ** (integer) is (-1)**(integer)*(+anything**integer)
 *      20. (-anything except 0 and inf) ** (non-integer) is NAN
 *
 * Accuracy:
 *      pow(x,y) returns x**y nearly rounded. In particular
 *                      pow(integer,integer)
 *      always returns the correct integer provided it is
 *      representable.
 *
 * Constants :
 * The hexadecimal values are the intended ones for the following
 * constants. The decimal values may be used, provided that the
 * compiler will convert from decimal to binary accurately enough
 * to produce the hexadecimal values shown.
 */
double q_pow(double x, double y);


/* __ieee754_sqrt(x)
 * Return correctly rounded sqrt.
 *           ------------------------------------------
 *	     |  Use the hardware sqrt if you have one |
 *           ------------------------------------------
 * Method:
 *   Bit by bit method using integer arithmetic. (Slow, but portable)
 *   1. Normalization
 *	Scale x to y in [1,4) with even powers of 2:
 *	find an integer k such that  1 <= (y=x*2^(2k)) < 4, then
 *		sqrt(x) = 2^k * sqrt(y)
 *   2. Bit by bit computation
 *	Let q  = sqrt(y) truncated to i bit after binary point (q = 1),
 *	     i							 0
 *                                     i+1         2
 *	    s  = 2*q , and	y  =  2   * ( y - q  ).		(1)
 *	     i      i            i                 i
 *
 *	To compute q    from q , one checks whether
 *		    i+1       i
 *
 *			      -(i+1) 2
 *			(q + 2      ) <= y.			(2)
 *     			  i
 *							      -(i+1)
 *	If (2) is false, then q   = q ; otherwise q   = q  + 2      .
 *		 	       i+1   i             i+1   i
 *
 *	With some algebric manipulation, it is not difficult to see
 *	that (2) is equivalent to
 *                             -(i+1)
 *			s  +  2       <= y			(3)
 *			 i                i
 *
 *	The advantage of (3) is that s  and y  can be computed by
 *				      i      i
 *	the following recurrence formula:
 *	    if (3) is false
 *
 *	    s     =  s  ,	y    = y   ;			(4)
 *	     i+1      i		 i+1    i
 *
 *	    otherwise,
 *                         -i                     -(i+1)
 *	    s	  =  s  + 2  ,  y    = y  -  s  - 2  		(5)
 *           i+1      i          i+1    i     i
 *
 *	One may easily use induction to prove (4) and (5).
 *	Note. Since the left hand side of (3) contain only i+2 bits,
 *	      it does not necessary to do a full (53-bit) comparison
 *	      in (3).
 *   3. Final rounding
 *	After generating the 53 bits result, we compute one more bit.
 *	Together with the remainder, we can decide whether the
 *	result is exact, bigger than 1/2ulp, or less than 1/2ulp
 *	(it will never equal to 1/2ulp).
 *	The rounding mode can be detected by checking whether
 *	huge + tiny is equal to huge, and whether huge - tiny is
 *	equal to huge for some floating point number "huge" and "tiny".
 *
 * Special cases:
 *	sqrt(+-0) = +-0 	... exact
 *	sqrt(inf) = inf
 *	sqrt(-ve) = NaN		... with invalid signal
 *	sqrt(NaN) = NaN		... with invalid signal for signaling NaN
 *
 */
double q_sqrt(double x);

/*
 * scalbn(double x, int n)
 * scalbn(x,n) returns x * 2**n computed by exponent
 * manipulation rather than by actually performing an
 * exponentiation or a multiplication.
 */
double q_scalbn(double x, int n);

/*
 * copysign(double x, double y)
 * copysign(x,y) returns a value with the magnitude of x and
 * with the sign bit of y.
 */
double q_copysign(double x, double y);


/* __ieee754_hypot(x,y)
 *
 * Method :
 *	If (assume round-to-nearest) z=x*x+y*y
 *	has error less than sqrt(2)/2 ulp, than
 *	sqrt(z) has error less than 1 ulp (exercise).
 *
 *	So, compute sqrt(x*x+y*y) with some care as
 *	follows to get the error below 1 ulp:
 *
 *	Assume x>y>0;
 *	(if possible, set rounding to round-to-nearest)
 *	1. if x > 2y  use
 *		x1*x1+(y*y+(x2*(x+x1))) for x*x+y*y
 *	where x1 = x with lower 32 bits cleared, x2 = x-x1; else
 *	2. if x <= 2y use
 *		t1*y1+((x-y)*(x-y)+(t1*y2+t2*y))
 *	where t1 = 2x with lower 32 bits cleared, t2 = 2x-t1,
 *	y1= y with lower 32 bits chopped, y2 = y-y1.
 *
 *	NOTE: scaling may be necessary if some argument is too
 *	      large or too tiny
 *
 * Special cases:
 *	hypot(x,y) is INF if x or y is +INF or -INF; else
 *	hypot(x,y) is NAN if x or y is NAN.
 *
 * Accuracy:
 * 	hypot(x,y) returns sqrt(x^2+y^2) with error less
 * 	than 1 ulps (units in the last place)
 */
double q_hypot(double x, double y);


#endif


/* vi: set et sw=2 sts=2: */
