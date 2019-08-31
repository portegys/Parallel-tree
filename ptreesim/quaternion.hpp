/*
 * This software is provided under the terms of the GNU General
 * Public License as published by the Free Software Foundation.
 *
 * Copyright (c) 2003 Tom Portegys, All Rights Reserved.
 * Permission to use, copy, modify, and distribute this software
 * and its documentation for NON-COMMERCIAL purposes and without
 * fee is hereby granted provided that this copyright notice
 * appears in all copies.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.
 */

//***************************************************************************//
//* File Name: quaternion.hpp                                               *//
//* Author:    Tom Portegys, portegys@ilstu.edu                             *//
//* Date Made: 07/25/02                                                     *//
//* File Desc: Class declaration and implementation details                 *//
//*            representing a quaternion.                                   *//
//* Rev. Date:                                                              *//
//* Rev. Desc:                                                              *//
//*                                                                         *//
//***************************************************************************//

#ifndef __QUATERNION_HPP__
#define __QUATERNION_HPP__

#ifdef UNIX
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#else
#include <windows.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#endif

class Quaternion
{
public:

   // Quaternion.
   float quat[4];

   // Constructor.
   Quaternion(float q[4])
   {
      quat[0] = q[0];
      quat[1] = q[1];
      quat[2] = q[2];
      quat[3] = q[3];
   }


   Quaternion()
   {
      quat[0] = 0.0f;
      quat[1] = 0.0f;
      quat[2] = 0.0f;
      quat[3] = 0.0f;
   }


   // Vector operations.
   void vzero(float *);
   void vset(float *, float, float, float);
   void vsub(const float *, const float *, float *);
   void vcopy(const float *, float *);
   void vcross(const float *, const float *, float *);
   float vlength(const float *);
   void vscale(float *, float);
   void vnormal(float *);
   float vdot(const float *, const float *);
   void vadd(const float *, const float *, float *);

   // Given an axis and angle, compute quaternion.
   void axis_to_quat(float a[3], float phi, float q[4]);

   // Add quaternions.
   void add_quats(float q1[4], float q2[4], float dest[4]);

   // Multiply quaternions.
   void mult_quats(float q1[4], float q2[4], float dest[4]);

   // Normalize a quaternion.
   void normalize_quat(float q[4]);

   // Build a rotation matrix, given a quaternion rotation.
   void build_rotmatrix(float m[4][4], float q[4]);

   static Quaternion MakeQFromEulerAngles(float x, float y, float z);
   static Vector MakeEulerAnglesFromQ(Quaternion q);

   static float const pi;

   static inline float DegreesToRadians(float deg)
   {
      return(deg * pi / 180.0f);
   }


   static inline float RadiansToDegrees(float rad)
   {
      return(rad * 180.0f / pi);
   }


private:

   int NormalCount;                               // Normalization counter.
};

float const Quaternion::pi = 3.14159265f;

void Quaternion::vzero(float *v)
{
   v[0] = 0.0;
   v[1] = 0.0;
   v[2] = 0.0;
}


void Quaternion::vset(float *v, float x, float y, float z)
{
   v[0] = x;
   v[1] = y;
   v[2] = z;
}


void Quaternion::vsub(const float *src1, const float *src2, float *dst)
{
   dst[0] = src1[0] - src2[0];
   dst[1] = src1[1] - src2[1];
   dst[2] = src1[2] - src2[2];
}


void Quaternion::vcopy(const float *v1, float *v2)
{
   register int i;

   for (i = 0; i < 3; i++)
   {
      v2[i] = v1[i];
   }
}


void Quaternion::vcross(const float *v1, const float *v2, float *cross)
{
   float temp[3];

   temp[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
   temp[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
   temp[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);

   vcopy(temp, cross);
}


float Quaternion::vlength(const float *v)
{
   return((float)sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]));
}


void Quaternion::vscale(float *v, float div)
{
   v[0] *= div;
   v[1] *= div;
   v[2] *= div;
}


void Quaternion::vnormal(float *v)
{
   vscale(v, 1.0f / vlength(v));
}


float Quaternion::vdot(const float *v1, const float *v2)
{
   return(v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]);
}


void Quaternion::vadd(const float *src1, const float *src2, float *dst)
{
   dst[0] = src1[0] + src2[0];
   dst[1] = src1[1] + src2[1];
   dst[2] = src1[2] + src2[2];
}


/*
 *  Given an axis and angle, compute quaternion.
 */
void Quaternion::axis_to_quat(float a[3], float phi, float q[4])
{
   vnormal(a);
   vcopy(a, q);
   vscale(q, (float)sin(phi / 2.0f));
   q[3] = (float)cos(phi / 2.0f);
}


/*
 * Add quaternions, normalizing periodically.
 */

#define NORMALFREQ    100

void Quaternion::add_quats(float q1[4], float q2[4], float dest[4])
{
   float t1[4], t2[4], t3[4];
   float tf[4];

   vcopy(q1, t1);
   vscale(t1, q2[3]);
   vcopy(q2, t2);
   vscale(t2, q1[3]);
   vcross(q2, q1, t3);
   vadd(t1, t2, tf);
   vadd(t3, tf, tf);
   tf[3]   = q1[3] * q2[3] - vdot(q1, q2);
   dest[0] = tf[0];
   dest[1] = tf[1];
   dest[2] = tf[2];
   dest[3] = tf[3];

   if (++NormalCount > NORMALFREQ)
   {
      NormalCount = 0;
      normalize_quat(dest);
   }
}


/*
 * Multiply quaternions, normalizing periodically.
 */
void Quaternion::mult_quats(float q1[4], float q2[4], float dest[4])
{
   dest[3] = q2[3] * q1[3]
             - q2[0] * q1[0]
             - q2[1] * q1[1]
             - q2[2] * q1[2];

   dest[0] = q2[3] * q1[0]
             + q2[0] * q1[3]
             + q2[1] * q1[2]
             - q2[2] * q1[1];

   dest[1] = q2[3] * q1[1]
             - q2[0] * q1[2]
             + q2[1] * q1[3]
             + q2[2] * q1[0];

   dest[2] = q2[3] * q1[2]
             + q2[0] * q1[1]
             - q2[1] * q1[0]
             + q2[2] * q1[3];

   if (++NormalCount > NORMALFREQ)
   {
      NormalCount = 0;
      normalize_quat(dest);
   }
}


/*
 * Quaternions always obey:  a^2 + b^2 + c^2 + d^2 = 1.0
 * If they don't add up to 1.0, dividing by their magnitude will
 * renormalize them.
 *
 * Note: See the following for more information on quaternions:
 *
 * - Shoemake, K., Animating rotation with quaternion curves, Computer
 *   Graphics 19, No 3 (Proc. SIGGRAPH'85), 245-254, 1985.
 * - Pletinckx, D., Quaternion calculus as a basic tool in computer
 *   graphics, The Visual Computer 5, 2-13, 1989.
 */
void Quaternion::normalize_quat(float q[4])
{
   int   i;
   float mag;

   mag = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
   for (i = 0; i < 4; i++)
   {
      q[i] /= mag;
   }
}


/*
 * Build a rotation matrix, given a quaternion rotation.
 */
void Quaternion::build_rotmatrix(float m[4][4], float q[4])
{
   m[0][0] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
   m[0][1] = 2.0f * (q[0] * q[1] - q[2] * q[3]);
   m[0][2] = 2.0f * (q[2] * q[0] + q[1] * q[3]);
   m[0][3] = 0.0f;

   m[1][0] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
   m[1][1] = 1.0f - 2.0f * (q[2] * q[2] + q[0] * q[0]);
   m[1][2] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
   m[1][3] = 0.0f;

   m[2][0] = 2.0f * (q[2] * q[0] - q[1] * q[3]);
   m[2][1] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
   m[2][2] = 1.0f - 2.0f * (q[1] * q[1] + q[0] * q[0]);
   m[2][3] = 0.0f;

   m[3][0] = 0.0f;
   m[3][1] = 0.0f;
   m[3][2] = 0.0f;
   m[3][3] = 1.0f;
}


Quaternion Quaternion::MakeQFromEulerAngles(float x, float y, float z)
{
   Quaternion q;
   double     roll  = (double)x;
   double     pitch = (double)y;
   double     yaw   = (double)z;

   double cyaw, cpitch, croll, syaw, spitch, sroll;
   double cyawcpitch, syawspitch, cyawspitch, syawcpitch;

   cyaw   = cos(0.5f * yaw);
   cpitch = cos(0.5f * pitch);
   croll  = cos(0.5f * roll);
   syaw   = sin(0.5f * yaw);
   spitch = sin(0.5f * pitch);
   sroll  = sin(0.5f * roll);

   cyawcpitch = cyaw * cpitch;
   syawspitch = syaw * spitch;
   cyawspitch = cyaw * spitch;
   syawcpitch = syaw * cpitch;

   q.quat[3] = (float)(cyawcpitch * croll + syawspitch * sroll);
   q.quat[0] = (float)(cyawcpitch * sroll - syawspitch * croll);
   q.quat[1] = (float)(cyawspitch * croll + syawcpitch * sroll);
   q.quat[2] = (float)(syawcpitch * croll - cyawspitch * sroll);

   return(q);
}


Vector Quaternion::MakeEulerAnglesFromQ(Quaternion q)
{
   double r11, r21, r31, r32, r33, r12, r13;
   double q00, q11, q22, q33;
   double tmp;
   Vector u;

   q00 = q.quat[3] * q.quat[3];
   q11 = q.quat[0] * q.quat[0];
   q22 = q.quat[1] * q.quat[1];
   q33 = q.quat[2] * q.quat[2];

   r11 = q00 + q11 - q22 - q33;
   r21 = 2 * (q.quat[0] * q.quat[1] + q.quat[3] * q.quat[2]);
   r31 = 2 * (q.quat[0] * q.quat[2] - q.quat[3] * q.quat[1]);
   r32 = 2 * (q.quat[1] * q.quat[2] + q.quat[3] * q.quat[0]);
   r33 = q00 - q11 - q22 + q33;

   tmp = fabs(r31);
   if (tmp > 0.999999)
   {
      r12 = 2 * (q.quat[0] * q.quat[1] - q.quat[3] * q.quat[2]);
      r13 = 2 * (q.quat[0] * q.quat[2] + q.quat[3] * q.quat[1]);

      u.x = 0.0f;                       //roll
      // pitch
      u.y = (float)(-(pi / 2) * r31 / tmp);
      // yaw
      u.z = (float)atan2(-r12, -r31 * r13);
      return(u);
   }

   // roll
   u.x = (float)atan2(r32, r33);
   // pitch
   u.y = (float)asin(-r31);
   // yaw
   u.z = (float)atan2(r21, r11);
   return(u);
}


#endif
