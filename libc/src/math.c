/*                    The Quest Operating System
 *  Copyright (C) 2005-2010  Richard West, Boston University
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

#include "math.h"
#include "types.h"


static const double huge = 1.0e300;

/* copied from uClibc s_round.c */
double q_round(double x){

  sint32 i0, j0;
  uint32 i1;

  union{
    double value;
    uint32 data[2];
  }temp;

  temp.value = x;
  i0 = temp.data[1];
  i1 = temp.data[0];

  j0 = ((i0 >> 20) & 0x7ff) - 0x3ff;
  if (j0 < 20)
    {
      if (j0 < 0)
        {
          if (huge + x > 0.0)
            {
              i0 &= 0x80000000;
              if (j0 == -1)
                i0 |= 0x3ff00000;
              i1 = 0;
            }
        }
      else
        {
          uint32 i = 0x000fffff >> j0;
          if (((i0 & i) | i1) == 0)
            /* X is integral.  */
            return x;
          if (huge + x > 0.0)
            {
              /* Raise inexact if x != 0.  */
              i0 += 0x00080000 >> j0;
              i0 &= ~i;
              i1 = 0;
            }
        }
    }
  else if (j0 > 51)
    {
      if (j0 == 0x400)
        /* Inf or NaN.  */
        return x + x;
      else
        return x;
    }
  else
    {
      uint32 i = 0xffffffff >> (j0 - 20);
      if ((i1 & i) == 0)
        /* X is integral.  */
        return x;

      if (huge + x > 0.0)
        {
          /* Raise inexact if x != 0.  */
          uint32 j = i1 + (1 << (51 - j0));
          if (j < i1)
            i0 += 1;
          i1 = j;
        }
      i1 &= ~i;
    }

  temp.data[1] = i0;
  temp.data[0] = i1;
  x = temp.value;

  return x;
}

double q_fabs(double t){
  
  if(t < 0) return -t;
  else return t;
}

float q_fabsf(float t){
  
  if(t < 0) return -t;
  else return t;
}

int q_lroundf(float x){

  return x >= 0 ? (int)(x + 0.5) : ((int)(x + 0.5)) - 1;
}


static const double
bp[] = {1.0, 1.5,},
dp_h[] = { 0.0, 5.84962487220764160156e-01,}, /* 0x3FE2B803, 0x40000000 */
dp_l[] = { 0.0, 1.35003920212974897128e-08,}, /* 0x3E4CFDEB, 0x43CFD006 */
zero    =  0.0,
one     =  1.0,
two     =  2.0,
two53   =  9007199254740992.0,  /* 0x43400000, 0x00000000 */
tiny    =  1.0e-300,
        /* poly coefs for (3/2)*(log(x)-2s-2/3*s**3 */
L1  =  5.99999999999994648725e-01, /* 0x3FE33333, 0x33333303 */
L2  =  4.28571428578550184252e-01, /* 0x3FDB6DB6, 0xDB6FABFF */
L3  =  3.33333329818377432918e-01, /* 0x3FD55555, 0x518F264D */
L4  =  2.72728123808534006489e-01, /* 0x3FD17460, 0xA91D4101 */
L5  =  2.30660745775561754067e-01, /* 0x3FCD864A, 0x93C9DB65 */
L6  =  2.06975017800338417784e-01, /* 0x3FCA7E28, 0x4A454EEF */
P1   =  1.66666666666666019037e-01, /* 0x3FC55555, 0x5555553E */
P2   = -2.77777777770155933842e-03, /* 0xBF66C16C, 0x16BEBD93 */
P3   =  6.61375632143793436117e-05, /* 0x3F11566A, 0xAF25DE2C */
P4   = -1.65339022054652515390e-06, /* 0xBEBBBD41, 0xC5D26BF1 */
P5   =  4.13813679705723846039e-08, /* 0x3E663769, 0x72BEA4D0 */
lg2  =  6.93147180559945286227e-01, /* 0x3FE62E42, 0xFEFA39EF */
lg2_h  =  6.93147182464599609375e-01, /* 0x3FE62E43, 0x00000000 */
lg2_l  = -1.90465429995776804525e-09, /* 0xBE205C61, 0x0CA86C39 */
ovt =  8.0085662595372944372e-0017, /* -(1024-log2(ovfl+.5ulp)) */
cp    =  9.61796693925975554329e-01, /* 0x3FEEC709, 0xDC3A03FD =2/(3ln2) */
cp_h  =  9.61796700954437255859e-01, /* 0x3FEEC709, 0xE0000000 =(float)cp */
cp_l  = -7.02846165095275826516e-09, /* 0xBE3E2FE0, 0x145B01F5 =tail of cp_h*/
ivln2    =  1.44269504088896338700e+00, /* 0x3FF71547, 0x652B82FE =1/ln2 */
ivln2_h  =  1.44269502162933349609e+00, /* 0x3FF71547, 0x60000000 =24b 1/ln2*/
ivln2_l  =  1.92596299112661746887e-08; /* 0x3E54AE0B, 0xF85DDF44 =1/ln2 tail*/

/* copied from uClibc e_pow.c, assumes Quest use IEEE 754 floating-point arithmetic */
double q_pow(double x, double y)
{/*
  int *tmp = ((int *) &x);
  printf("%x %x\n", *tmp, *(tmp + 1));
  tmp = ((int *) &y);
  printf("%x %x\n", *tmp, *(tmp + 1));
*/
  double z,ax,z_h,z_l,p_h,p_l;
  double y1,t1,t2,r,s,t,u,v,w;
  sint32 i,j,k,yisint,n;
  sint32 hx,hy,ix,iy;
  uint32 lx,ly;

  union{
    double value;
    uint32 data[2];
  } temp;

  temp.value = x;
  hx = temp.data[1];
  lx = temp.data[0];
  /* x==1: 1**y = 1 (even if y is NaN) */
  if(hx==0x3ff00000 && lx==0) {
    return x;
  }
  ix = hx&0x7fffffff;

  temp.value = y;
  hy = temp.data[1];
  ly = temp.data[0];
  iy = hy&0x7fffffff;
 
//  printf("hx:%x, lx:%x, ix:%x\n", hx, lx, ix);
//  printf("hy:%x, ly:%x, iy:%x\n", hy, ly, iy);

  /* y==zero: x**0 = 1 */
  if((iy|ly)==0) {return one;}

  /* +-NaN return x+y */
  if(ix > 0x7ff00000 || ((ix==0x7ff00000)&&(lx!=0)) ||
    iy > 0x7ff00000 || ((iy==0x7ff00000)&&(ly!=0)))
    {return x+y;}

  /* determine if y is an odd int when x < 0
   * yisint = 0       ... y is not an integer
   * yisint = 1       ... y is an odd int
   * yisint = 2       ... y is an even int
   */
  yisint  = 0;
  if(hx<0) {
    if(iy>=0x43400000) yisint = 2; /* even integer y */
    else if(iy>=0x3ff00000) {
      k = (iy>>20)-0x3ff;        /* exponent */
      if(k>20) {
        j = ly>>(52-k);
        if((j<<(52-k))==ly) yisint = 2-(j&1);
      } 
      else if(ly==0) {
        j = iy>>(20-k);
        if((j<<(20-k))==iy) yisint = 2-(j&1);
      }
    }
  }

  /* special value of y */
  if(ly==0) {
    if(iy==0x7ff00000) {       /* y is +-inf */
      if (((ix-0x3ff00000)|lx)==0)
        return one;         /* +-1**+-inf is 1 (yes, weird rule) */
      if (ix >= 0x3ff00000)   /* (|x|>1)**+-inf = inf,0 */
        return (hy>=0) ? y : zero;
      /* (|x|<1)**-,+inf = inf,0 */
      return (hy<0) ? -y : zero;
    }
    if(iy==0x3ff00000) {        /* y is  +-1 */
      if(hy<0) return one/x; else return x;
    }
    if(hy==0x40000000) return x*x; /* y is  2 */
    if(hy==0x3fe00000) {        /* y is  0.5 */
      if(hx>=0)       /* x >= +0 */
        return q_sqrt(x);
    }
  }

  ax = q_fabs(x);
  /* special value of x */
  if(lx==0) {
    if(ix==0x7ff00000||ix==0||ix==0x3ff00000){
      z = ax;                 /*x is +-0,+-inf,+-1*/
      if(hy<0) z = one/z;     /* z = (1/|x|) */
      if(hx<0) {
        if(((ix-0x3ff00000)|yisint)==0) {
          z = (z-z)/(z-z); /* (-1)**non-int is NaN */
        } 
        else if(yisint==1)
          z = -z;         /* (x<0)**odd = -(|x|**odd) */
      }
      return z;
    }
  }

  /* (x<0)**(non-int) is NaN */
  if(((((uint32)hx>>31)-1)|yisint)==0) return (x-x)/(x-x);

  /* |y| is huge */
  if(iy>0x41e00000) { /* if |y| > 2**31 */
    if(iy>0x43f00000){  /* if |y| > 2**64, must o/uflow */
      if(ix<=0x3fefffff) return (hy<0)? huge*huge:tiny*tiny;
      if(ix>=0x3ff00000) return (hy>0)? huge*huge:tiny*tiny;
    }
    /* over/underflow if x is not close to one */
    if(ix<0x3fefffff) return (hy<0)? huge*huge:tiny*tiny;
    if(ix>0x3ff00000) return (hy>0)? huge*huge:tiny*tiny;
    /* now |1-x| is tiny <= 2**-20, suffice to compute
    log(x) by x-x^2/2+x^3/3-x^4/4 */
    t = x-1;            /* t has 20 trailing zeros */
    w = (t*t)*(0.5-t*(0.3333333333333333333333-t*0.25));
    u = ivln2_h*t;      /* ivln2_h has 21 sig. bits */
    v = t*ivln2_l-w*ivln2;
    t1 = u+v;
    temp.value = t1;
    temp.data[0] = 0;
    t1 = temp.value;
    t2 = v-(t1-u);
  } 
  else {
    double s2,s_h,s_l,t_h,t_l;
    n = 0;
    /* take care subnormal number */
    if(ix<0x00100000) {
      ax *= two53; 
      n -= 53; 
      temp.value = ax;
      ix = temp.data[1]; 
    }
    n  += ((ix)>>20)-0x3ff;
    j  = ix&0x000fffff;
    /* determine interval */
    ix = j|0x3ff00000;          /* normalize ix */
    if(j<=0x3988E) k=0;         /* |x|<sqrt(3/2) */
    else if(j<0xBB67A) k=1;     /* |x|<sqrt(3)   */
    else {k=0;n+=1;ix -= 0x00100000;}
    temp.value = ax;
    temp.data[1] = ix;
    ax = temp.value;

    /* compute s = s_h+s_l = (x-1)/(x+1) or (x-1.5)/(x+1.5) */
    u = ax-bp[k];               /* bp[0]=1.0, bp[1]=1.5 */
    v = one/(ax+bp[k]);
    s = u*v;
    s_h = s;
    temp.value = s_h;
    temp.data[0] = 0;
    s_h = temp.value;
    /* t_h=ax+bp[k] High */
    t_h = zero;
    temp.value = t_h;
    temp.data[1] = ((ix>>1)|0x20000000)+0x00080000+(k<<18);
    t_h = temp.value;
    t_l = ax - (t_h-bp[k]);
    s_l = v*((u-s_h*t_h)-s_h*t_l);
    /* compute log(ax) */
    s2 = s*s;
    r = s2*s2*(L1+s2*(L2+s2*(L3+s2*(L4+s2*(L5+s2*L6)))));
    r += s_l*(s_h+s);
    s2  = s_h*s_h;
    t_h = 3.0+s2+r;
    temp.value = t_h;
    temp.data[0] = 0;
    t_h = temp.value;
    t_l = r-((t_h-3.0)-s2);
    /* u+v = s*(1+...) */
    u = s_h*t_h;
    v = s_l*t_h+t_l*s;
    /* 2/(3log2)*(s+...) */
    p_h = u+v;
    temp.value = p_h;
    temp.data[0] = 0;
    p_h = temp.value;
    p_l = v-(p_h-u);
    z_h = cp_h*p_h;             /* cp_h+cp_l = 2/(3*log2) */
    z_l = cp_l*p_h+p_l*cp+dp_l[k];
    /* log2(ax) = (s+..)*2/(3*log2) = n + dp_h + z_h + z_l */
    t = (double)n;
    t1 = (((z_h+z_l)+dp_h[k])+t);
    temp.value = t1;
    temp.data[0] = 0;
    t1 = temp.value;
    t2 = z_l-(((t1-t)-dp_h[k])-z_h);
  }

  s = one; /* s (sign of result -ve**odd) = -1 else = 1 */
  if(((((uint32)hx>>31)-1)|(yisint-1))==0)
    s = -one;/* (-ve)**(odd int) */

  /* split up y into y1+y2 and compute (y1+y2)*(t1+t2) */
  y1  = y;
  temp.value = y1;
  temp.data[0] = 0;
  y1 = temp.value;
  p_l = (y-y1)*t1+y*t2;
  p_h = y1*t1;
  z = p_l+p_h;
  temp.value = z;
  j = temp.data[1];
  i = temp.data[0];
  if (j>=0x40900000) {                            /* z >= 1024 */
    if(((j-0x40900000)|i)!=0)                   /* if z > 1024 */
      return s*huge*huge;                     /* overflow */
    else {
      if(p_l+ovt>z-p_h) return s*huge*huge;   /* overflow */
    }
  } 
  else if((j&0x7fffffff)>=0x4090cc00 ) {        /* z <= -1075 */
    if(((j-0xc090cc00)|i)!=0)           /* z < -1075 */
      return s*tiny*tiny;             /* underflow */
    else {
      if(p_l<=z-p_h) return s*tiny*tiny;      /* underflow */
    }
  }
  /*
   * compute 2**(p_h+p_l)
   */
  i = j&0x7fffffff;
  k = (i>>20)-0x3ff;
  n = 0;
  if(i>0x3fe00000) {              /* if |z| > 0.5, set n = [z+0.5] */
    n = j+(0x00100000>>(k+1));
    k = ((n&0x7fffffff)>>20)-0x3ff;     /* new k for n */
    t = zero;
    temp.value = t;
    temp.data[1] = n&~(0x000fffff>>k);
    t = temp.value;
    n = ((n&0x000fffff)|0x00100000)>>(20-k);
    if(j<0) n = -n;
    p_h -= t;
  }
  t = p_l+p_h;
  temp.value = t;
  temp.data[0] = 0;
  t = temp.value;
  u = t*lg2_h;
  v = (p_l-(t-p_h))*lg2+t*lg2_l;
  z = u+v;
  w = v-(z-u);
  t  = z*z;
  t1  = z - t*(P1+t*(P2+t*(P3+t*(P4+t*P5))));
  r  = (z*t1)/(t1-two)-(w+z*w);
  z  = one-(r-z);
  temp.value = z;
  j = temp.data[1];
  j += (n<<20);
  if((j>>20)<=0) z = q_scalbn(z,n); /* subnormal output */
  else{
    temp.value = z;
    temp.data[1] = j;
    z = temp.value;
  }  
  return s*z;
}


/* copied from uClibc e_sqrt.c, assumes Quest use IEEE 754 floating-point arithmetic */
double q_sqrt(double x)
{
  double z;
  sint32 sign = (int)0x80000000;
  sint32 ix0,s0,q,m,t,i;
  uint32 r,t1,s1,ix1,q1;

  union{
    double value;
    uint32 data[2];
  }temp;

  temp.value = x;
  ix0 = temp.data[1];
  ix1 = temp.data[0];

  /* take care of Inf and NaN */
  if((ix0&0x7ff00000)==0x7ff00000) {
    return x*x+x;		/* sqrt(NaN)=NaN, sqrt(+inf)=+inf
					   sqrt(-inf)=sNaN */
  }
  /* take care of zero */
  if(ix0<=0) {
    if(((ix0&(~sign))|ix1)==0) return x;/* sqrt(+-0) = +-0 */
    else if(ix0<0)
	return (x-x)/(x-x);		/* sqrt(-ve) = sNaN */
  }
  /* normalize x */
  m = (ix0>>20);
  if(m==0) {				/* subnormal x */
    while(ix0==0) {
      m -= 21;
      ix0 |= (ix1>>11); ix1 <<= 21;
    }
    for(i=0;(ix0&0x00100000)==0;i++) ix0<<=1;
    m -= i-1;
    ix0 |= (ix1>>(32-i));
    ix1 <<= i;
  }
  m -= 1023;	/* unbias exponent */
  ix0 = (ix0&0x000fffff)|0x00100000;
  if(m&1){	/* odd m, double x to make it even */
    ix0 += ix0 + ((ix1&sign)>>31);
    ix1 += ix1;
  }
  m >>= 1;	/* m = [m/2] */

  /* generate sqrt(x) bit by bit */
  ix0 += ix0 + ((ix1&sign)>>31);
  ix1 += ix1;
  q = q1 = s0 = s1 = 0;	/* [q,q1] = sqrt(x) */
  r = 0x00200000;		/* r = moving bit from right to left */

  while(r!=0) {
    t = s0+r;
    if(t<=ix0) {
      s0 = t+r;
      ix0 -= t;
      q += r;
    }
    ix0 += ix0 + ((ix1&sign)>>31);
    ix1 += ix1;
    r>>=1;
  }

  r = sign;
  while(r!=0) {
    t1 = s1+r;
    t  = s0;
    if((t<ix0)||((t==ix0)&&(t1<=ix1))) {
      s1  = t1+r;
      if(((t1&sign)==sign)&&(s1&sign)==0) s0 += 1;
      ix0 -= t;
      if (ix1 < t1) ix0 -= 1;
      ix1 -= t1;
      q1  += r;
    }
    ix0 += ix0 + ((ix1&sign)>>31);
    ix1 += ix1;
    r>>=1;
  }

  /* use floating add to find out rounding direction */
  if((ix0|ix1)!=0) {
    z = one-tiny; /* trigger inexact flag */
    if (z>=one) {
      z = one+tiny;
      if (q1==(uint32)0xffffffff) { q1=0; q += 1;}
      else if (z>one) {
        if (q1==(uint32)0xfffffffe) q+=1;
	q1+=2;
      } else
        q1 += (q1&1);
    }
  }
  ix0 = (q>>1)+0x3fe00000;
  ix1 =  q1>>1;
  if ((q&1)==1) ix1 |= sign;
  ix0 += (m <<20);
  temp.data[1] = ix0;
  temp.data[0] = ix1;
  z = temp.value;
  return z;
}


static const double
two54  = 1.80143985094819840000e+16, /* 0x43500000, 0x00000000 */
twom54 = 5.55111512312578270212e-17; /* 0x3C900000, 0x00000000 */

/* copied from uClibc s_scalbn.c, assumes sizeof(long) == sizeof(int) */
double q_scalbn(double x, int n)
{
  sint32 k, hx, lx;

  union{
    double value;
    uint32 data[2];
  }temp;

  temp.value = x;
  hx = temp.data[1];
  lx = temp.data[0];
  k = (hx & 0x7ff00000) >> 20; /* extract exponent */
  if (k == 0) { /* 0 or subnormal x */
    if ((lx | (hx & 0x7fffffff)) == 0)
      return x; /* +-0 */
    x *= two54;
    temp.value = x;
    hx = temp.data[1];
    k = ((hx & 0x7ff00000) >> 20) - 54;
  }
  if (k == 0x7ff)
    return x + x; /* NaN or Inf */
  k = k + n;
  if (k > 0x7fe)
    return huge * q_copysign(huge, x); /* overflow */
  if (n < -50000)
    return tiny * q_copysign(tiny, x); /* underflow */
  if (k > 0) { /* normal result */
    temp.value = x;
    temp.data[1] = (hx & 0x800fffff) | (k << 20);
    x = temp.value;
    return x;
  }
  if (k <= -54) {
    if (n > 50000) /* in case integer overflow in n+k */
      return huge * q_copysign(huge, x); /* overflow */
      return tiny * q_copysign(tiny, x); /* underflow */
  }
  k += 54; /* subnormal result */
  temp.value = x;
  temp.data[1] = (hx & 0x800fffff) | (k << 20);
  x = temp.value;
  return x * twom54;
}

/* copied from uClibc s_copysign.c */
double q_copysign(double x, double y)
{
  uint32 hx, hy;
  union{
    double value;
    uint32 data[2];
  }temp;

  temp.value = x;
  hx = temp.data[1];
  temp.value = y;
  hy = temp.data[1];
  temp.value = x;
  temp.data[1] = (hx&0x7fffffff)|(hy&0x80000000);
  x = temp.value;
  return x;
}

/* copied from uClibc e_hypot.c, assumes Quest use IEEE 754 floating-point arithmetic */
double q_hypot(double x, double y)
{
  double a=x,b=y,t1,t2,y1,y2,w;
  sint32 j,k,ha,hb;

  union{
    double value;
    uint32 data[2];
  }temp;

  temp.value = x;
  ha = temp.data[1];
  ha &= 0x7fffffff;
  temp.value = y;
  hb = temp.data[1];
  hb &= 0x7fffffff;
  if(hb > ha) {a=y;b=x;j=ha; ha=hb;hb=j;} else {a=x;b=y;}
  /* a <- |a| */
  temp.value = a;
  temp.data[1] = ha;
  a = temp.value;
  /* b <- |b| */
  temp.value = b;
  temp.data[1] = hb;
  b = temp.value;
  if((ha-hb)>0x3c00000) {return a+b;} /* x/y > 2**60 */
  k=0;
  if(ha > 0x5f300000) {	/* a>2**500 */
    if(ha >= 0x7ff00000) {	/* Inf or NaN */
      uint32 low;
      w = a+b;			/* for sNaN */
      temp.value = a;
      low = temp.data[0];
      if(((ha&0xfffff)|low)==0) w = a;
      temp.value = b;
      low = temp.data[0];
      if(((hb^0x7ff00000)|low)==0) w = b;
      return w;
    }
    /* scale a and b by 2**-600 */
    ha -= 0x25800000; hb -= 0x25800000;	k += 600;
    temp.value = a;
    temp.data[1] = ha;
    a = temp.value;
    temp.value = b;
    temp.data[1] = hb;
    b = temp.value;
  }
  if(hb < 0x20b00000) {	/* b < 2**-500 */
    if(hb <= 0x000fffff) {	/* subnormal b or 0 */
      uint32 low;
      temp.value = b;
      low = temp.data[0];
      if((hb|low)==0) return a;
      t1=0;
      /* t1=2^1022 */
      temp.value = t1;
      temp.data[1] = 0x7fd00000;
      t1 = temp.value;
      b *= t1;
      a *= t1;
      k -= 1022;
    } 
    else {		/* scale a and b by 2^600 */
      ha += 0x25800000; 	/* a *= 2^600 */
      hb += 0x25800000;	/* b *= 2^600 */
      k -= 600;
      temp.value = a;
      temp.data[1] = ha;
      a = temp.value;
      temp.value = b;
      temp.data[1] = hb;
      b = temp.value;
    }
  }
  /* medium size a and b */
  w = a-b;
  if (w>b) {
    t1 = 0;
    temp.value = t1;
    temp.data[1] = ha;
    t1 = temp.value;
    t2 = a-t1;
    w = q_sqrt(t1*t1-(b*(-b)-t2*(a+t1)));
  } 
  else {
    a  = a+a;
    y1 = 0;
    temp.value = y1;
    temp.data[1] = hb;
    y1 = temp.value;
    y2 = b - y1;
    t1 = 0;
    temp.value = t1;
    temp.data[1] = ha+0x00100000;
    t1 = temp.value;
    t2 = a - t1;
    w = q_sqrt(t1*y1-(w*(-w)-(t1*y2+t2*b)));
  }
  if(k!=0) {
    uint32 high;
    t1 = 1.0;
    temp.value = t1;
    high = temp.data[1];
    temp.value = t1;
    temp.data[1] = high+(k<<20);
    t1 = temp.value;
    return t1*w;
  } 
  else return w;
}




/* vi: set et sw=2 sts=2: */
