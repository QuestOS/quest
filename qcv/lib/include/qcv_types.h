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

#ifndef _QCV_QCV_TYPES_H_
#define _QCV_QCV_TYPES_H_


#define HAVE_BOOLEAN            /* So libjpeg doesn't complain */
typedef enum { FALSE = 0, TRUE = 1 } boolean;

#ifndef BOOL
#define BOOL boolean
#endif

#ifndef max
#define max(a,b)                \
  ({ __typeof__ (a) _a = (a);   \
    __typeof__ (b) _b = (b);    \
    _a > _b ? _a : _b; })
#endif

#ifndef min
#define min(a,b)                \
  ({ __typeof__ (a) _a = (a);   \
     __typeof__ (b) _b = (b);   \
    _a < _b ? _a : _b; })
#endif


/* The following is taken from opencv the following is the license
   from that file */

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/


#define QCV_CN_MAX     512
#define QCV_CN_SHIFT   3
#define QCV_DEPTH_MAX  (1 << QCV_CN_SHIFT)

#define QCV_8U   0
#define QCV_8S   1
#define QCV_16U  2
#define QCV_16S  3
#define QCV_32S  4
#define QCV_32F  5
#define QCV_64F  6
#define QCV_USRTYPE1 7

#define QCV_MAT_DEPTH_MASK       (QCV_DEPTH_MAX - 1)
#define QCV_MAT_DEPTH(flags)     ((flags) & QCV_MAT_DEPTH_MASK)

#define QCV_MAKETYPE(depth,cn) (QCV_MAT_DEPTH(depth) + (((cn)-1) << QCV_CN_SHIFT))
#define QCV_MAKE_TYPE QCV_MAKETYPE

#define QCV_8UC1 QCV_MAKETYPE(QCV_8U,1)
#define QCV_8UC2 QCV_MAKETYPE(QCV_8U,2)
#define QCV_8UC3 QCV_MAKETYPE(QCV_8U,3)
#define QCV_8UC4 QCV_MAKETYPE(QCV_8U,4)
#define QCV_8UC(n) QCV_MAKETYPE(QCV_8U,(n))

#define QCV_8SC1 QCV_MAKETYPE(QCV_8S,1)
#define QCV_8SC2 QCV_MAKETYPE(QCV_8S,2)
#define QCV_8SC3 QCV_MAKETYPE(QCV_8S,3)
#define QCV_8SC4 QCV_MAKETYPE(QCV_8S,4)
#define QCV_8SC(n) QCV_MAKETYPE(QCV_8S,(n))

#define QCV_16UC1 QCV_MAKETYPE(QCV_16U,1)
#define QCV_16UC2 QCV_MAKETYPE(QCV_16U,2)
#define QCV_16UC3 QCV_MAKETYPE(QCV_16U,3)
#define QCV_16UC4 QCV_MAKETYPE(QCV_16U,4)
#define QCV_16UC(n) QCV_MAKETYPE(QCV_16U,(n))

#define QCV_16SC1 QCV_MAKETYPE(QCV_16S,1)
#define QCV_16SC2 QCV_MAKETYPE(QCV_16S,2)
#define QCV_16SC3 QCV_MAKETYPE(QCV_16S,3)
#define QCV_16SC4 QCV_MAKETYPE(QCV_16S,4)
#define QCV_16SC(n) QCV_MAKETYPE(QCV_16S,(n))

#define QCV_32SC1 QCV_MAKETYPE(QCV_32S,1)
#define QCV_32SC2 QCV_MAKETYPE(QCV_32S,2)
#define QCV_32SC3 QCV_MAKETYPE(QCV_32S,3)
#define QCV_32SC4 QCV_MAKETYPE(QCV_32S,4)
#define QCV_32SC(n) QCV_MAKETYPE(QCV_32S,(n))

#define QCV_32FC1 QCV_MAKETYPE(QCV_32F,1)
#define QCV_32FC2 QCV_MAKETYPE(QCV_32F,2)
#define QCV_32FC3 QCV_MAKETYPE(QCV_32F,3)
#define QCV_32FC4 QCV_MAKETYPE(QCV_32F,4)
#define QCV_32FC(n) QCV_MAKETYPE(QCV_32F,(n))

#define QCV_64FC1 QCV_MAKETYPE(QCV_64F,1)
#define QCV_64FC2 QCV_MAKETYPE(QCV_64F,2)
#define QCV_64FC3 QCV_MAKETYPE(QCV_64F,3)
#define QCV_64FC4 QCV_MAKETYPE(QCV_64F,4)
#define QCV_64FC(n) QCV_MAKETYPE(QCV_64F,(n))

#define QCV_MAT_CN_MASK          ((QCV_CN_MAX - 1) << QCV_CN_SHIFT)
#define QCV_MAT_CN(flags)        ((((flags) & QCV_MAT_CN_MASK) >> QCV_CN_SHIFT) + 1)
#define QCV_MAT_TYPE_MASK        (QCV_DEPTH_MAX*QCV_CN_MAX - 1)
#define QCV_MAT_TYPE(flags)      ((flags) & QCV_MAT_TYPE_MASK)
#define QCV_MAT_CONT_FLAG_SHIFT  14
#define QCV_MAT_CONT_FLAG        (1 << QCV_MAT_CONT_FLAG_SHIFT)
#define QCV_IS_MAT_CONT(flags)   ((flags) & QCV_MAT_CONT_FLAG)
#define QCV_IS_CONT_MAT          QCV_IS_MAT_CONT
#define QCV_SUBMAT_FLAG_SHIFT    15
#define QCV_SUBMAT_FLAG          (1 << QCV_SUBMAT_FLAG_SHIFT)
#define QCV_IS_SUBMAT(flags)     ((flags) & QCV_MAT_SUBMAT_FLAG)


#endif // _QCV_QCV_TYPES_H_

/*
 * Local Variables:
 * indent-tabs-mode: nil
 * mode: C
 * c-file-style: "gnu"
 * c-basic-offset: 2
 * End:
 */

/* vi: set et sw=2 sts=2: */
