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

#include "math.h"



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


/*
 * __kernel_cos( x,  y )
 * kernel cos function on [-pi/4, pi/4], pi/4 ~ 0.785398164
 * Input x is assumed to be bounded by ~pi/4 in magnitude.
 * Input y is the tail of x.
 *
 * Algorithm
 *	1. Since cos(-x) = cos(x), we need only to consider positive x.
 *	2. if x < 2^-27 (hx<0x3e400000 0), return 1 with inexact if x!=0.
 *	3. cos(x) is approximated by a polynomial of degree 14 on
 *	   [0,pi/4]
 *		  	                 4            14
 *	   	cos(x) ~ 1 - x*x/2 + C1*x + ... + C6*x
 *	   where the remez error is
 *
 * 	|              2     4     6     8     10    12     14 |     -58
 * 	|cos(x)-(1-.5*x +C1*x +C2*x +C3*x +C4*x +C5*x  +C6*x  )| <= 2
 * 	|    					               |
 *
 * 	               4     6     8     10    12     14
 *	4. let r = C1*x +C2*x +C3*x +C4*x +C5*x  +C6*x  , then
 *	       cos(x) = 1 - x*x/2 + r
 *	   since cos(x+y) ~ cos(x) - sin(x)*y
 *			  ~ cos(x) - x*y,
 *	   a correction term is necessary in cos(x) and hence
 *		cos(x+y) = 1 - (x*x/2 - (r - x*y))
 *	   For better accuracy when x > 0.3, let qx = |x|/4 with
 *	   the last 32 bits mask off, and if x > 0.78125, let qx = 0.28125.
 *	   Then
 *		cos(x+y) = (1-qx) - ((x*x/2-qx) - (r-x*y)).
 *	   Note that 1-qx and (x*x/2-qx) is EXACT here, and the
 *	   magnitude of the latter is at least a quarter of x*x/2,
 *	   thus, reducing the rounding error in the subtraction.
 */
static const double
C1  =  4.16666666666666019037e-02, /* 0x3FA55555, 0x5555554C */
C2  = -1.38888888888741095749e-03, /* 0xBF56C16C, 0x16C15177 */
C3  =  2.48015872894767294178e-05, /* 0x3EFA01A0, 0x19CB1590 */
C4  = -2.75573143513906633035e-07, /* 0xBE927E4F, 0x809C52AD */
C5  =  2.08757232129817482790e-09, /* 0x3E21EE9E, 0xBDB4B1C4 */
C6  = -1.13596475577881948265e-11; /* 0xBDA8FAE9, 0xBE8838D4 */

/* copied from uClibc k_cos.c */
double   __kernel_cos(double x, double y)
{
  double a,hz,z,r,qx;
  sint32 ix;
  GET_HIGH_WORD(ix,x);
  ix &= 0x7fffffff;			/* ix = |x|'s high word*/
  if(ix<0x3e400000) {			/* if x < 2**27 */
    if(((int)x)==0) return one;		/* generate inexact */
  }
  z  = x*x;
  r  = z*(C1+z*(C2+z*(C3+z*(C4+z*(C5+z*C6)))));
  if(ix < 0x3FD33333) 			/* if |x| < 0.3 */
    return one - (0.5*z - (z*r - x*y));
  else {
    if(ix > 0x3fe90000) {		/* x > 0.78125 */
      qx = 0.28125;
    } else {
      INSERT_WORDS(qx,ix-0x00200000,0);	/* x/4 */
    }
    hz = 0.5*z-qx;
    a  = one-qx;
    return a - (hz - (z*r-x*y));
  }
}

/* __kernel_sin( x, y, iy)
 * kernel sin function on [-pi/4, pi/4], pi/4 ~ 0.7854
 * Input x is assumed to be bounded by ~pi/4 in magnitude.
 * Input y is the tail of x.
 * Input iy indicates whether y is 0. (if iy=0, y assume to be 0).
 *
 * Algorithm
 *	1. Since sin(-x) = -sin(x), we need only to consider positive x.
 *	2. if x < 2^-27 (hx<0x3e400000 0), return x with inexact if x!=0.
 *	3. sin(x) is approximated by a polynomial of degree 13 on
 *	   [0,pi/4]
 *		  	         3            13
 *	   	sin(x) ~ x + S1*x + ... + S6*x
 *	   where
 *
 * 	|sin(x)         2     4     6     8     10     12  |     -58
 * 	|----- - (1+S1*x +S2*x +S3*x +S4*x +S5*x  +S6*x   )| <= 2
 * 	|  x 					           |
 *
 *	4. sin(x+y) = sin(x) + sin'(x')*y
 *		    ~ sin(x) + (1-x*x/2)*y
 *	   For better accuracy, let
 *		     3      2      2      2      2
 *		r = x *(S2+x *(S3+x *(S4+x *(S5+x *S6))))
 *	   then                   3    2
 *		sin(x) = x + (S1*x + (x *(r-y/2)+y))
 */
static const double
half =  5.00000000000000000000e-01, /* 0x3FE00000, 0x00000000 */
S1  = -1.66666666666666324348e-01, /* 0xBFC55555, 0x55555549 */
S2  =  8.33333333332248946124e-03, /* 0x3F811111, 0x1110F8A6 */
S3  = -1.98412698298579493134e-04, /* 0xBF2A01A0, 0x19C161D5 */
S4  =  2.75573137070700676789e-06, /* 0x3EC71DE3, 0x57B1FE7D */
S5  = -2.50507602534068634195e-08, /* 0xBE5AE5E6, 0x8A2B9CEB */
S6  =  1.58969099521155010221e-10; /* 0x3DE5D93A, 0x5ACFD57C */


/* copied from uClibc k_sin.c */
double   __kernel_sin(double x, double y, int iy)
{
  double z,r,v;
  sint32 ix;
  GET_HIGH_WORD(ix,x);
  ix &= 0x7fffffff;			/* high word of x */
  if(ix<0x3e400000)			/* |x| < 2**-27 */
  {if((int)x==0) return x;}		/* generate inexact */
  z	=  x*x;
  v	=  z*x;
  r	=  S2+z*(S3+z*(S4+z*(S5+z*S6)));
  if(iy==0) return x+v*(S1+z*r);
  else      return x-((z*(half*y-v*r)-y)-v*S1);
}


double q_floor(double x)
{
  sint32 i0,i1,j0;
  uint32 i,j;
  EXTRACT_WORDS(i0,i1,x);
  j0 = ((i0>>20)&0x7ff)-0x3ff;
  if(j0<20) {
    if(j0<0) { 	/* raise inexact if x != 0 */
      if(huge+x>0.0) {/* return 0*sign(x) if |x|<1 */
        if(i0>=0) {i0=i1=0;}
        else if(((i0&0x7fffffff)|i1)!=0)
        { i0=0xbff00000;i1=0;}
      }
    } else {
      i = (0x000fffff)>>j0;
      if(((i0&i)|i1)==0) return x; /* x is integral */
      if(huge+x>0.0) {	/* raise inexact flag */
        if(i0<0) i0 += (0x00100000)>>j0;
        i0 &= (~i); i1=0;
      }
    }
  } else if (j0>51) {
    if(j0==0x400) return x+x;	/* inf or NaN */
    else return x;		/* x is integral */
  } else {
    i = ((uint32)(0xffffffff))>>(j0-20);
    if((i1&i)==0) return x;	/* x is integral */
    if(huge+x>0.0) { 		/* raise inexact flag */
      if(i0<0) {
        if(j0==20) i0+=1;
        else {
          j = i1+(1<<(52-j0));
          if(j<i1) i0 +=1 ; 	/* got a carry */
          i1=j;
        }
      }
      i1 &= (~i);
    }
  }
  INSERT_WORDS(x,i0,i1);
  return x;
}



/*
 * __kernel_rem_pio2(x,y,e0,nx,prec,ipio2)
 * double x[],y[]; int e0,nx,prec; int ipio2[];
 *
 * __kernel_rem_pio2 return the last three digits of N with
 *		y = x - N*pi/2
 * so that |y| < pi/2.
 *
 * The method is to compute the integer (mod 8) and fraction parts of
 * (2/pi)*x without doing the full multiplication. In general we
 * skip the part of the product that are known to be a huge integer (
 * more accurately, = 0 mod 8 ). Thus the number of operations are
 * independent of the exponent of the input.
 *
 * (2/pi) is represented by an array of 24-bit integers in ipio2[].
 *
 * Input parameters:
 * 	x[]	The input value (must be positive) is broken into nx
 *		pieces of 24-bit integers in double precision format.
 *		x[i] will be the i-th 24 bit of x. The scaled exponent
 *		of x[0] is given in input parameter e0 (i.e., x[0]*2^e0
 *		match x's up to 24 bits.
 *
 *		Example of breaking a double positive z into x[0]+x[1]+x[2]:
 *			e0 = ilogb(z)-23
 *			z  = scalbn(z,-e0)
 *		for i = 0,1,2
 *			x[i] = floor(z)
 *			z    = (z-x[i])*2**24
 *
 *
 *	y[]	ouput result in an array of double precision numbers.
 *		The dimension of y[] is:
 *			24-bit  precision	1
 *			53-bit  precision	2
 *			64-bit  precision	2
 *			113-bit precision	3
 *		The actual value is the sum of them. Thus for 113-bit
 *		precison, one may have to do something like:
 *
 *		long double t,w,r_head, r_tail;
 *		t = (long double)y[2] + (long double)y[1];
 *		w = (long double)y[0];
 *		r_head = t+w;
 *		r_tail = w - (r_head - t);
 *
 *	e0	The exponent of x[0]
 *
 *	nx	dimension of x[]
 *
 *  	prec	an integer indicating the precision:
 *			0	24  bits (single)
 *			1	53  bits (double)
 *			2	64  bits (extended)
 *			3	113 bits (quad)
 *
 *	ipio2[]
 *		integer array, contains the (24*i)-th to (24*i+23)-th
 *		bit of 2/pi after binary point. The corresponding
 *		floating value is
 *
 *			ipio2[i] * 2^(-24(i+1)).
 *
 * External function:
 *	double scalbn(), floor();
 *
 *
 * Here is the description of some local variables:
 *
 * 	jk	jk+1 is the initial number of terms of ipio2[] needed
 *		in the computation. The recommended value is 2,3,4,
 *		6 for single, double, extended,and quad.
 *
 * 	jz	local integer variable indicating the number of
 *		terms of ipio2[] used.
 *
 *	jx	nx - 1
 *
 *	jv	index for pointing to the suitable ipio2[] for the
 *		computation. In general, we want
 *			( 2^e0*x[0] * ipio2[jv-1]*2^(-24jv) )/8
 *		is an integer. Thus
 *			e0-3-24*jv >= 0 or (e0-3)/24 >= jv
 *		Hence jv = max(0,(e0-3)/24).
 *
 *	jp	jp+1 is the number of terms in PIo2[] needed, jp = jk.
 *
 * 	q[]	double array with integral value, representing the
 *		24-bits chunk of the product of x and 2/pi.
 *
 *	q0	the corresponding exponent of q[0]. Note that the
 *		exponent for q[i] would be q0-24*i.
 *
 *	PIo2[]	double precision array, obtained by cutting pi/2
 *		into 24 bits chunks.
 *
 *	f[]	ipio2[] in floating point
 *
 *	iq[]	integer array by breaking up q[] in 24-bits chunk.
 *
 *	fq[]	final product of x*(2/pi) in fq[0],..,fq[jk]
 *
 *	ih	integer. If >0 it indicates q[] is >= 0.5, hence
 *		it also indicates the *sign* of the result.
 *
 */


/*
 * Constants:
 * The hexadecimal values are the intended ones for the following
 * constants. The decimal values may be used, provided that the
 * compiler will convert from decimal to binary accurately enough
 * to produce the hexadecimal values shown.
 */
static const int init_jk[] = {2,3,4,6}; /* initial value for jk */

static const double PIo2[] = {
  1.57079625129699707031e+00, /* 0x3FF921FB, 0x40000000 */
  7.54978941586159635335e-08, /* 0x3E74442D, 0x00000000 */
  5.39030252995776476554e-15, /* 0x3CF84698, 0x80000000 */
  3.28200341580791294123e-22, /* 0x3B78CC51, 0x60000000 */
  1.27065575308067607349e-29, /* 0x39F01B83, 0x80000000 */
  1.22933308981111328932e-36, /* 0x387A2520, 0x40000000 */
  2.73370053816464559624e-44, /* 0x36E38222, 0x80000000 */
  2.16741683877804819444e-51, /* 0x3569F31D, 0x00000000 */
};

static const double
twon24  =  5.96046447753906250000e-08, /* 0x3E700000, 0x00000000 */
two24   =  1.67772160000000000000e+07; /* 0x41700000, 0x00000000 */

/* copied from uClibc k_rem_pio2.c */
int   __kernel_rem_pio2(double *x, double *y, int e0, int nx, int prec, const sint32 *ipio2)
{
  sint32 jz,jx,jv,jp,jk,carry,n,iq[20],i,j,k,m,q0,ih;
  double z,fw,f[20],fq[20],q[20];

  /* initialize jk*/
  jk = init_jk[prec];
  jp = jk;

  /* determine jx,jv,q0, note that 3>q0 */
  jx =  nx-1;
  jv = (e0-3)/24; if(jv<0) jv=0;
  q0 =  e0-24*(jv+1);

  /* set up f[0] to f[jx+jk] where f[jx+jk] = ipio2[jv+jk] */
  j = jv-jx; m = jx+jk;
  for(i=0;i<=m;i++,j++) f[i] = (j<0)? zero : (double) ipio2[j];

  /* compute q[0],q[1],...q[jk] */
  for (i=0;i<=jk;i++) {
    for(j=0,fw=0.0;j<=jx;j++) fw += x[j]*f[jx+i-j]; q[i] = fw;
  }

  jz = jk;
recompute:
  /* distill q[] into iq[] reversingly */
  for(i=0,j=jz,z=q[jz];j>0;i++,j--) {
    fw    =  (double)((sint32)(twon24* z));
    iq[i] =  (sint32)(z-two24*fw);
    z     =  q[j-1]+fw;
  }

  /* compute n */
  z  = q_scalbn(z,q0);		/* actual value of z */
  z -= 8.0*q_floor(z*0.125);		/* trim off integer >= 8 */
  n  = (sint32) z;
  z -= (double)n;
  ih = 0;
  if(q0>0) {	/* need iq[jz-1] to determine n */
    i  = (iq[jz-1]>>(24-q0)); n += i;
    iq[jz-1] -= i<<(24-q0);
    ih = iq[jz-1]>>(23-q0);
  }
  else if(q0==0) ih = iq[jz-1]>>23;
  else if(z>=0.5) ih=2;

  if(ih>0) {	/* q > 0.5 */
    n += 1; carry = 0;
    for(i=0;i<jz ;i++) {	/* compute 1-q */
      j = iq[i];
      if(carry==0) {
        if(j!=0) {
          carry = 1; iq[i] = 0x1000000- j;
        }
      } else  iq[i] = 0xffffff - j;
    }
    if(q0>0) {		/* rare case: chance is 1 in 12 */
      switch(q0) {
        case 1:
          iq[jz-1] &= 0x7fffff; break;
        case 2:
          iq[jz-1] &= 0x3fffff; break;
      }
    }
    if(ih==2) {
      z = one - z;
      if(carry!=0) z -= q_scalbn(one,q0);
    }
  }

  /* check if recomputation is needed */
  if(z==zero) {
    j = 0;
    for (i=jz-1;i>=jk;i--) j |= iq[i];
    if(j==0) { /* need recomputation */
      for(k=1;iq[jk-k]==0;k++);   /* k = no. of terms needed */

      for(i=jz+1;i<=jz+k;i++) {   /* add q[jz+1] to q[jz+k] */
        f[jx+i] = (double) ipio2[jv+i];
        for(j=0,fw=0.0;j<=jx;j++) fw += x[j]*f[jx+i-j];
        q[i] = fw;
      }
      jz += k;
      goto recompute;
    }
  }

  /* chop off zero terms */
  if(z==0.0) {
    jz -= 1; q0 -= 24;
    while(iq[jz]==0) { jz--; q0-=24;}
  } else { /* break z into 24-bit if necessary */
    z = q_scalbn(z,-q0);
    if(z>=two24) {
      fw = (double)((sint32)(twon24*z));
      iq[jz] = (sint32)(z-two24*fw);
      jz += 1; q0 += 24;
      iq[jz] = (sint32) fw;
    } else iq[jz] = (sint32) z ;
  }

  /* convert integer "bit" chunk to floating-point value */
  fw = q_scalbn(one,q0);
  for(i=jz;i>=0;i--) {
    q[i] = fw*(double)iq[i]; fw*=twon24;
  }

  /* compute PIo2[0,...,jp]*q[jz,...,0] */
  for(i=jz;i>=0;i--) {
    for(fw=0.0,k=0;k<=jp&&k<=jz-i;k++) fw += PIo2[k]*q[i+k];
    fq[jz-i] = fw;
  }

  /* compress fq[] into y[] */
  switch(prec) {
    case 0:
      fw = 0.0;
      for (i=jz;i>=0;i--) fw += fq[i];
      y[0] = (ih==0)? fw: -fw;
      break;
    case 1:
    case 2:
      fw = 0.0;
      for (i=jz;i>=0;i--) fw += fq[i];
      y[0] = (ih==0)? fw: -fw;
      fw = fq[0]-fw;
      for (i=1;i<=jz;i++) fw += fq[i];
      y[1] = (ih==0)? fw: -fw;
      break;
    case 3:	/* painful */
      for (i=jz;i>0;i--) {
        fw      = fq[i-1]+fq[i];
        fq[i]  += fq[i-1]-fw;
        fq[i-1] = fw;
      }
      for (i=jz;i>1;i--) {
        fw      = fq[i-1]+fq[i];
        fq[i]  += fq[i-1]-fw;
        fq[i-1] = fw;
      }
      for (fw=0.0,i=jz;i>=2;i--) fw += fq[i];
      if(ih==0) {
        y[0] =  fq[0]; y[1] =  fq[1]; y[2] =  fw;
      } else {
        y[0] = -fq[0]; y[1] = -fq[1]; y[2] = -fw;
      }
  }
  return n&7;
}



/* __ieee754_rem_pio2(x,y)
 *
 * return the remainder of x rem pi/2 in y[0]+y[1]
 * use __kernel_rem_pio2()
 */
/*
 * Table of constants for 2/pi, 396 Hex digits (476 decimal) of 2/pi
 */
static const sint32 two_over_pi[] = {
0xA2F983, 0x6E4E44, 0x1529FC, 0x2757D1, 0xF534DD, 0xC0DB62,
0x95993C, 0x439041, 0xFE5163, 0xABDEBB, 0xC561B7, 0x246E3A,
0x424DD2, 0xE00649, 0x2EEA09, 0xD1921C, 0xFE1DEB, 0x1CB129,
0xA73EE8, 0x8235F5, 0x2EBB44, 0x84E99C, 0x7026B4, 0x5F7E41,
0x3991D6, 0x398353, 0x39F49C, 0x845F8B, 0xBDF928, 0x3B1FF8,
0x97FFDE, 0x05980F, 0xEF2F11, 0x8B5A0A, 0x6D1F6D, 0x367ECF,
0x27CB09, 0xB74F46, 0x3F669E, 0x5FEA2D, 0x7527BA, 0xC7EBE5,
0xF17B3D, 0x0739F7, 0x8A5292, 0xEA6BFB, 0x5FB11F, 0x8D5D08,
0x560330, 0x46FC7B, 0x6BABF0, 0xCFBC20, 0x9AF436, 0x1DA9E3,
0x91615E, 0xE61B08, 0x659985, 0x5F14A0, 0x68408D, 0xFFD880,
0x4D7327, 0x310606, 0x1556CA, 0x73A8C9, 0x60E27B, 0xC08C6B,
};

static const sint32 npio2_hw[] = {
0x3FF921FB, 0x400921FB, 0x4012D97C, 0x401921FB, 0x401F6A7A, 0x4022D97C,
0x4025FDBB, 0x402921FB, 0x402C463A, 0x402F6A7A, 0x4031475C, 0x4032D97C,
0x40346B9C, 0x4035FDBB, 0x40378FDB, 0x403921FB, 0x403AB41B, 0x403C463A,
0x403DD85A, 0x403F6A7A, 0x40407E4C, 0x4041475C, 0x4042106C, 0x4042D97C,
0x4043A28C, 0x40446B9C, 0x404534AC, 0x4045FDBB, 0x4046C6CB, 0x40478FDB,
0x404858EB, 0x404921FB,
};

/*
 * invpio2:  53 bits of 2/pi
 * pio2_1:   first  33 bit of pi/2
 * pio2_1t:  pi/2 - pio2_1
 * pio2_2:   second 33 bit of pi/2
 * pio2_2t:  pi/2 - (pio2_1+pio2_2)
 * pio2_3:   third  33 bit of pi/2
 * pio2_3t:  pi/2 - (pio2_1+pio2_2+pio2_3)
 */

static const double
invpio2 =  6.36619772367581382433e-01, /* 0x3FE45F30, 0x6DC9C883 */
pio2_1  =  1.57079632673412561417e+00, /* 0x3FF921FB, 0x54400000 */
pio2_1t =  6.07710050650619224932e-11, /* 0x3DD0B461, 0x1A626331 */
pio2_2  =  6.07710050630396597660e-11, /* 0x3DD0B461, 0x1A600000 */
pio2_2t =  2.02226624879595063154e-21, /* 0x3BA3198A, 0x2E037073 */
pio2_3  =  2.02226624871116645580e-21, /* 0x3BA3198A, 0x2E000000 */
pio2_3t =  8.47842766036889956997e-32; /* 0x397B839A, 0x252049C1 */

/* copied from uClibc e_rem_pio2.c */
sint32   __ieee754_rem_pio2(double x, double *y)
{
  double z=0.0,w,t,r,fn;
  double tx[3];
  sint32 e0,i,j,nx,n,ix,hx;
  uint32 low;

  GET_HIGH_WORD(hx,x);		/* high word of x */
  ix = hx&0x7fffffff;
  if(ix<=0x3fe921fb)   /* |x| ~<= pi/4 , no need for reduction */
  {y[0] = x; y[1] = 0; return 0;}
  if(ix<0x4002d97c) {  /* |x| < 3pi/4, special case with n=+-1 */
    if(hx>0) {
      z = x - pio2_1;
      if(ix!=0x3ff921fb) { 	/* 33+53 bit pi is good enough */
        y[0] = z - pio2_1t;
        y[1] = (z-y[0])-pio2_1t;
      } else {		/* near pi/2, use 33+33+53 bit pi */
        z -= pio2_2;
        y[0] = z - pio2_2t;
        y[1] = (z-y[0])-pio2_2t;
      }
      return 1;
    } else {	/* negative x */
      z = x + pio2_1;
      if(ix!=0x3ff921fb) { 	/* 33+53 bit pi is good enough */
        y[0] = z + pio2_1t;
        y[1] = (z-y[0])+pio2_1t;
      } else {		/* near pi/2, use 33+33+53 bit pi */
        z += pio2_2;
        y[0] = z + pio2_2t;
        y[1] = (z-y[0])+pio2_2t;
      }
      return -1;
    }
  }
  if(ix<=0x413921fb) { /* |x| ~<= 2^19*(pi/2), medium size */
    t  = q_fabs(x);
    n  = (sint32) (t*invpio2+half);
    fn = (double)n;
    r  = t-fn*pio2_1;
    w  = fn*pio2_1t;	/* 1st round good to 85 bit */
    if(n<32&&ix!=npio2_hw[n-1]) {
      y[0] = r-w;	/* quick check no cancellation */
    } else {
      uint32 high;
      j  = ix>>20;
      y[0] = r-w;
      GET_HIGH_WORD(high,y[0]);
      i = j-((high>>20)&0x7ff);
      if(i>16) {  /* 2nd iteration needed, good to 118 */
        t  = r;
        w  = fn*pio2_2;
        r  = t-w;
        w  = fn*pio2_2t-((t-r)-w);
        y[0] = r-w;
        GET_HIGH_WORD(high,y[0]);
        i = j-((high>>20)&0x7ff);
        if(i>49)  {	/* 3rd iteration need, 151 bits acc */
          t  = r;	/* will cover all possible cases */
          w  = fn*pio2_3;
          r  = t-w;
          w  = fn*pio2_3t-((t-r)-w);
          y[0] = r-w;
        }
      }
    }
    y[1] = (r-y[0])-w;
    if(hx<0) 	{y[0] = -y[0]; y[1] = -y[1]; return -n;}
    else	 return n;
  }
  /*
   * all other (large) arguments
   */
  if(ix>=0x7ff00000) {		/* x is inf or NaN */
    y[0]=y[1]=x-x; return 0;
  }
  /* set z = scalbn(|x|,ilogb(x)-23) */
  GET_LOW_WORD(low,x);
  SET_LOW_WORD(z,low);
  e0 	= (ix>>20)-1046;	/* e0 = ilogb(z)-23; */
  SET_HIGH_WORD(z, ix - ((sint32)(e0<<20)));
  for(i=0;i<2;i++) {
    tx[i] = (double)((sint32)(z));
    z     = (z-tx[i])*two24;
  }
  tx[2] = z;
  nx = 3;
  while(tx[nx-1]==zero) nx--;	/* skip zero term */
  n  =  __kernel_rem_pio2(tx,y,e0,nx,2,two_over_pi);
  if(hx<0) {y[0] = -y[0]; y[1] = -y[1]; return -n;}
  return n;
}

/* copied from uClibc s_cos.c */
double q_cos(double x)
{
  double y[2],z=0.0;
  sint32 n, ix;

  /* High word of x. */
  GET_HIGH_WORD(ix,x);

  /* |x| ~< pi/4 */
  ix &= 0x7fffffff;
  if(ix <= 0x3fe921fb) return __kernel_cos(x,z);

  /* cos(Inf or NaN) is NaN */
  else if (ix>=0x7ff00000) return x-x;

  /* argument reduction needed */
  else {
    n = __ieee754_rem_pio2(x,y);
    switch(n&3) {
      case 0: return  __kernel_cos(y[0],y[1]);
      case 1: return -__kernel_sin(y[0],y[1],1);
      case 2: return -__kernel_cos(y[0],y[1]);
      default:
              return  __kernel_sin(y[0],y[1],1);
    }
  }
}


/* copied from uClibc s_sin.c */
double q_sin(double x)
{
  double y[2],z=0.0;
  sint32 n, ix;

  /* High word of x. */
  GET_HIGH_WORD(ix,x);

  /* |x| ~< pi/4 */
  ix &= 0x7fffffff;
  if(ix <= 0x3fe921fb) return __kernel_sin(x,z,0);

  /* sin(Inf or NaN) is NaN */
  else if (ix>=0x7ff00000) return x-x;

  /* argument reduction needed */
  else {
    n = __ieee754_rem_pio2(x,y);
    switch(n&3) {
      case 0: return  __kernel_sin(y[0],y[1],1);
      case 1: return  __kernel_cos(y[0],y[1]);
      case 2: return -__kernel_sin(y[0],y[1],1);
      default:
              return -__kernel_cos(y[0],y[1]);
    }
  }
}


/* __kernel_tan( x, y, k )
 * kernel tan function on [-pi/4, pi/4], pi/4 ~ 0.7854
 * Input x is assumed to be bounded by ~pi/4 in magnitude.
 * Input y is the tail of x.
 * Input k indicates whether tan (if k=1) or
 * -1/tan (if k= -1) is returned.
 *
 * Algorithm
 *	1. Since tan(-x) = -tan(x), we need only to consider positive x.
 *	2. if x < 2^-28 (hx<0x3e300000 0), return x with inexact if x!=0.
 *	3. tan(x) is approximated by a odd polynomial of degree 27 on
 *	   [0,0.67434]
 *		  	         3             27
 *	   	tan(x) ~ x + T1*x + ... + T13*x
 *	   where
 *
 * 	        |tan(x)         2     4            26   |     -59.2
 * 	        |----- - (1+T1*x +T2*x +.... +T13*x    )| <= 2
 * 	        |  x 					|
 *
 *	   Note: tan(x+y) = tan(x) + tan'(x)*y
 *		          ~ tan(x) + (1+x*x)*y
 *	   Therefore, for better accuracy in computing tan(x+y), let
 *		     3      2      2       2       2
 *		r = x *(T2+x *(T3+x *(...+x *(T12+x *T13))))
 *	   then
 *		 		    3    2
 *		tan(x+y) = x + (T1*x + (x *(r+y)+y))
 *
 *      4. For x in [0.67434,pi/4],  let y = pi/4 - x, then
 *		tan(x) = tan(pi/4-y) = (1-tan(y))/(1+tan(y))
 *		       = 1 - 2*(tan(y) - (tan(y)^2)/(1+tan(y)))
 */
static const double
pio4  =  7.85398163397448278999e-01, /* 0x3FE921FB, 0x54442D18 */
pio4lo=  3.06161699786838301793e-17, /* 0x3C81A626, 0x33145C07 */
T[] =  {
  3.33333333333334091986e-01, /* 0x3FD55555, 0x55555563 */
  1.33333333333201242699e-01, /* 0x3FC11111, 0x1110FE7A */
  5.39682539762260521377e-02, /* 0x3FABA1BA, 0x1BB341FE */
  2.18694882948595424599e-02, /* 0x3F9664F4, 0x8406D637 */
  8.86323982359930005737e-03, /* 0x3F8226E3, 0xE96E8493 */
  3.59207910759131235356e-03, /* 0x3F6D6D22, 0xC9560328 */
  1.45620945432529025516e-03, /* 0x3F57DBC8, 0xFEE08315 */
  5.88041240820264096874e-04, /* 0x3F4344D8, 0xF2F26501 */
  2.46463134818469906812e-04, /* 0x3F3026F7, 0x1A8D1068 */
  7.81794442939557092300e-05, /* 0x3F147E88, 0xA03792A6 */
  7.14072491382608190305e-05, /* 0x3F12B80F, 0x32F0A7E9 */
 -1.85586374855275456654e-05, /* 0xBEF375CB, 0xDB605373 */
  2.59073051863633712884e-05, /* 0x3EFB2A70, 0x74BF7AD4 */
};

/* copied from uClibc k_tan.c */
double   __kernel_tan(double x, double y, int iy)
{
  double z,r,v,w,s;
  sint32 ix,hx;
  GET_HIGH_WORD(hx,x);
  ix = hx&0x7fffffff;	/* high word of |x| */
  if(ix<0x3e300000)			/* x < 2**-28 */
  {if((int)x==0) {			/* generate inexact */
                   uint32 low;
                   GET_LOW_WORD(low,x);
                   if(((ix|low)|(iy+1))==0) return one/q_fabs(x);
                   else return (iy==1)? x: -one/x;
                 }
  }
  if(ix>=0x3FE59428) { 			/* |x|>=0.6744 */
    if(hx<0) {x = -x; y = -y;}
    z = pio4-x;
    w = pio4lo-y;
    x = z+w; y = 0.0;
  }
  z	=  x*x;
  w 	=  z*z;
  /* Break x^5*(T[1]+x^2*T[2]+...) into
   *	  x^5(T[1]+x^4*T[3]+...+x^20*T[11]) +
   *	  x^5(x^2*(T[2]+x^4*T[4]+...+x^22*[T12]))
   */
  r = T[1]+w*(T[3]+w*(T[5]+w*(T[7]+w*(T[9]+w*T[11]))));
  v = z*(T[2]+w*(T[4]+w*(T[6]+w*(T[8]+w*(T[10]+w*T[12])))));
  s = z*x;
  r = y + z*(s*(r+v)+y);
  r += T[0]*s;
  w = x+r;
  if(ix>=0x3FE59428) {
    v = (double)iy;
    return (double)(1-((hx>>30)&2))*(v-2.0*(x-(w*w/(w+v)-r)));
  }
  if(iy==1) return w;
  else {		/* if allow error up to 2 ulp,
                           simply return -1.0/(x+r) here */
    /*  compute -1.0/(x+r) accurately */
    double a,t;
    z  = w;
    SET_LOW_WORD(z,0);
    v  = r-(z - x); 	/* z+v = r+x */
    t = a  = -1.0/w;	/* a = -1.0/w */
    SET_LOW_WORD(t,0);
    s  = 1.0+t*z;
    return t+a*(s+t*v);
  }
}

/* copied from uClibc s_tan.c */
double q_tan(double x)
{
  double y[2],z=0.0;
  sint32 n, ix;

  /* High word of x. */
  GET_HIGH_WORD(ix,x);

  /* |x| ~< pi/4 */
  ix &= 0x7fffffff;
  if(ix <= 0x3fe921fb) return __kernel_tan(x,z,1);

  /* tan(Inf or NaN) is NaN */
  else if (ix>=0x7ff00000) return x-x;		/* NaN */

  /* argument reduction needed */
  else {
    n = __ieee754_rem_pio2(x,y);
    return __kernel_tan(y[0],y[1],1-((n&1)<<1)); /*   1 -- n even
                                                      -1 -- n odd */
  }
}


static const double
halF[2]	= {0.5,-0.5,},
twom1000= 9.33263618503218878990e-302,     /* 2**-1000=0x01700000,0*/
o_threshold=  7.09782712893383973096e+02,  /* 0x40862E42, 0xFEFA39EF */
u_threshold= -7.45133219101941108420e+02,  /* 0xc0874910, 0xD52D3051 */
ln2HI[2]   ={ 6.93147180369123816490e-01,  /* 0x3fe62e42, 0xfee00000 */
	     -6.93147180369123816490e-01,},/* 0xbfe62e42, 0xfee00000 */
ln2LO[2]   ={ 1.90821492927058770002e-10,  /* 0x3dea39ef, 0x35793c76 */
	     -1.90821492927058770002e-10,},/* 0xbdea39ef, 0x35793c76 */
invln2 =  1.44269504088896338700e+00; /* 0x3ff71547, 0x652b82fe */

/* copied from uClibc e_exp.c, assumes Quest use IEEE 754 floating-point arithmetic */
double q_exp(double x)	/* default IEEE double exp */
{
  double y;
  double hi = 0.0;
  double lo = 0.0;
  double c;
  double t;
  sint32 k=0;
  sint32 xsb;
  uint32 hx;

  GET_HIGH_WORD(hx,x);
  xsb = (hx>>31)&1;		/* sign bit of x */
  hx &= 0x7fffffff;		/* high word of |x| */

  /* filter out non-finite argument */
  if(hx >= 0x40862E42) {			/* if |x|>=709.78... */
    if(hx>=0x7ff00000) {
      uint32 lx;
      GET_LOW_WORD(lx,x);
      if(((hx&0xfffff)|lx)!=0)
        return x+x; 		/* NaN */
      else return (xsb==0)? x:0.0;	/* exp(+-inf)={inf,0} */
    }
    if(x > o_threshold) return huge*huge; /* overflow */
    if(x < u_threshold) return twom1000*twom1000; /* underflow */
  }

  /* argument reduction */
  if(hx > 0x3fd62e42) {		/* if  |x| > 0.5 ln2 */
    if(hx < 0x3FF0A2B2) {	/* and |x| < 1.5 ln2 */
      hi = x-ln2HI[xsb]; lo=ln2LO[xsb]; k = 1-xsb-xsb;
    } else {
      k  = invln2*x+halF[xsb];
      t  = k;
      hi = x - t*ln2HI[0];	/* t*ln2HI is exact here */
      lo = t*ln2LO[0];
    }
    x  = hi - lo;
  }
  else if(hx < 0x3e300000)  {	/* when |x|<2**-28 */
    if(huge+x>one) return one+x;/* trigger inexact */
  }
  else k = 0;

  /* x is now in primary range */
  t  = x*x;
  c  = x - t*(P1+t*(P2+t*(P3+t*(P4+t*P5))));
  if(k==0) 	return one-((x*c)/(c-2.0)-x);
  else 		y = one-((lo-(x*c)/(2.0-c))-hi);
  if(k >= -1021) {
    uint32 hy;
    GET_HIGH_WORD(hy,y);
    SET_HIGH_WORD(y,hy+(k<<20));	/* add k to y's exponent */
    return y;
  } else {
    uint32 hy;
    GET_HIGH_WORD(hy,y);
    SET_HIGH_WORD(y,hy+((k+1000)<<20));	/* add k to y's exponent */
    return y*twom1000;
  }
}



/* vi: set et sw=2 sts=2: */
