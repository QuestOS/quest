#ifndef __SOFT_FLOAT_H_
#define __SOFT_FLOAT_H_

/*
 * These functions are used by gcc when the flag -msoft-float is used
 * to do cpu floating point computations
 */

/*
 * These functions return the sum of a and b.
 */

float __addsf3 (float a, float b);
double __adddf3 (double a, double b);
long double __addtf3 (long double a, long double b);
long double __addxf3 (long double a, long double b);


/*
 * These functions return the difference between b and a; that is, a - b.
 */
  
float __subsf3 (float a, float b);
double __subdf3 (double a, double b);
long double __subtf3 (long double a, long double b);
long double __subxf3 (long double a, long double b);


/*
 * These functions return the product of a and b.
 */
  
float __mulsf3 (float a, float b);
double __muldf3 (double a, double b);
long double __multf3 (long double a, long double b);
long double __mulxf3 (long double a, long double b);


/*
 * These functions return the quotient of a and b; that is, a / b.
 */

float __divsf3 (float a, float b);
double __divdf3 (double a, double b);
long double __divtf3 (long double a, long double b);
long double __divxf3 (long double a, long double b);

/*
 * These functions return the negation of a. They simply flip the sign
 * bit, so they can produce negative zero and negative NaN.
 */

float __negsf2 (float a);
double __negdf2 (double a);
long double __negtf2 (long double a);
long double __negxf2 (long double a);


/*
 * These functions extend a to the wider mode of their return type.
 */

double __extendsfdf2 (float a);
long double __extendsftf2 (float a);
long double __extendsfxf2 (float a);
long double __extenddftf2 (double a);
long double __extenddfxf2 (double a);

/*
 * These functions truncate a to the narrower mode of their return
 * type, rounding toward zero.
 */
  
double __truncxfdf2 (long double a);
double __trunctfdf2 (long double a);
float __truncxfsf2 (long double a);
float __trunctfsf2 (long double a);
float __truncdfsf2 (double a);


/*
 * These functions convert a to a signed integer, rounding toward
 * zero.
 */
  
int __fixsfsi (float a);
int __fixdfsi (double a);
int __fixtfsi (long double a);
int __fixxfsi (long double a);


/*
 * These functions convert a to a signed long, rounding toward zero.
 */

long __fixsfdi (float a);
long __fixdfdi (double a);
long __fixtfdi (long double a);
long __fixxfdi (long double a);

/*
 * These functions convert a to a signed long long, rounding toward zero.
 */

long long __fixsfti (float a);
long long __fixdfti (double a);
long long __fixtfti (long double a);
long long __fixxfti (long double a);


/*
 * These functions convert a to an unsigned integer, rounding toward
 * zero. Negative values all become zero.
 */

unsigned int __fixunssfsi (float a);
unsigned int __fixunsdfsi (double a);
unsigned int __fixunstfsi (long double a);
unsigned int __fixunsxfsi (long double a);


/*
 * These functions convert a to an unsigned long, rounding toward
 * zero. Negative values all become zero.
 */

unsigned long __fixunssfdi (float a);
unsigned long __fixunsdfdi (double a);
unsigned long __fixunstfdi (long double a);
unsigned long __fixunsxfdi (long double a);

/*
 * These functions convert a to an unsigned long long, rounding toward
 * zero. Negative values all become zero.
 */

unsigned long long __fixunssfti (float a);
unsigned long long __fixunsdfti (double a);
unsigned long long __fixunstfti (long double a);
unsigned long long __fixunsxfti (long double a);


/*
 * These functions convert i, a signed integer, to floating point.
 */

float __floatsisf (int i);
double __floatsidf (int i);
long double __floatsitf (int i);
long double __floatsixf (int i);


/*
 * These functions convert i, a signed long, to floating point.
 */
float __floatdisf (long i);
double __floatdidf (long i);
long double __floatditf (long i);
long double __floatdixf (long i);

 
/*
 * These functions convert i, a signed long long, to floating point.
 */

float __floattisf (long long i);
double __floattidf (long long i);
long double __floattitf (long long i);
long double __floattixf (long long i);

/*
 * These functions convert i, an unsigned integer, to floating point.
 */

float __floatunsisf (unsigned int i);
double __floatunsidf (unsigned int i);
long double __floatunsitf (unsigned int i);
long double __floatunsixf (unsigned int i);


/*
 * These functions convert i, an unsigned long, to floating point.
 */

float __floatundisf (unsigned long i);
double __floatundidf (unsigned long i);
long double __floatunditf (unsigned long i);
long double __floatundixf (unsigned long i);


/*
 * These functions convert i, an unsigned long long, to floating
 * point.
 */

float __floatuntisf (unsigned long long i);
double __floatuntidf (unsigned long long i);
long double __floatuntitf (unsigned long long i);
long double __floatuntixf (unsigned long long i);


/*
 * These functions calculate a <=> b. That is, if a is less than b,
 * they return −1; if a is greater than b, they return 1; and if a and
 * b are equal they return 0. If either argument is NaN they return 1,
 * but you should not rely on this; if NaN is a possibility, use one
 * of the higher-level comparison functions.
 */

int __cmpsf2 (float a, float b);
int __cmpdf2 (double a, double b);
int __cmptf2 (long double a, long double b);


/*
 * These functions return a nonzero value if either argument is NaN,
 * otherwise 0.
 *
 * There is also a complete group of higher level functions which
 * correspond directly to comparison operators. They implement the ISO
 * C semantics for floating-point comparisons, taking NaN into
 * account. Pay careful attention to the return values defined for
 * each set. Under the hood, all of these routines are implemented as
 *
 *      if (__unordXf2 (a, b))
 *        return E;
 *      return __cmpXf2 (a, b);
 *      
 * where E is a constant chosen to give the proper behavior for
 * NaN. Thus, the meaning of the return value is different for each
 * set. Do not rely on this implementation; only the semantics documented
 * below are guaranteed.
 */


int __unordsf2 (float a, float b);
int __unorddf2 (double a, double b);
int __unordtf2 (long double a, long double b);



/*
 * These functions return zero if neither argument is NaN, and a and b
 * are equal.
 */

int __eqsf2 (float a, float b);
int __eqdf2 (double a, double b);
int __eqtf2 (long double a, long double b);

/*
 * These functions return a nonzero value if either argument is NaN,
 * or if a and b are unequal.
 */

int __nesf2 (float a, float b);
int __nedf2 (double a, double b);
int __netf2 (long double a, long double b);



/*
 * These functions return a value greater than or equal to zero if
 * neither argument is NaN, and a is greater than or equal to b.
 */

int __gesf2 (float a, float b);
int __gedf2 (double a, double b);
int __getf2 (long double a, long double b);


/*
 * These functions return a value less than zero if neither argument
 * is NaN, and a is strictly less than b.
 */

int __ltsf2 (float a, float b);
int __ltdf2 (double a, double b);
int __lttf2 (long double a, long double b);

/*
 * These functions return a value less than or equal to zero if
 * neither argument is NaN, and a is less than or equal to b.
 */

int __lesf2 (float a, float b);
int __ledf2 (double a, double b);
int __letf2 (long double a, long double b);


/*
 * These functions return a value greater than zero if neither
 * argument is NaN, and a is strictly greater than b.
 */
int __gtsf2 (float a, float b);
int __gtdf2 (double a, double b);
int __gttf2 (long double a, long double b);



#endif // __SOFT_FLOAT_H_
