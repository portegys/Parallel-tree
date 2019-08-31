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
//* File Name: spacial.hpp                                                  *//
//* Author:    Tom Portegys, portegys@ilstu.edu                             *//
//* Date Made: 07/25/02                                                     *//
//* File Desc: Class declaration and implementation details                 *//
//*            representing spacial properties: rotation, translation,      *//
//*            scale, and speed.                                            *//
//* Rev. Date:                                                              *//
//* Rev. Desc:                                                              *//
//*                                                                         *//
//***************************************************************************//

#ifndef __SPACIAL_HPP__
#define __SPACIAL_HPP__

#ifndef UNIX
#include <windows.h>
#endif
#include "quaternion.hpp"

// Pi
#ifndef UNIX
#define M_PI    3.14159265358979323846f
#endif

class cSpacial
{
public:

   // Rotation.
   float pitch, yaw, roll;                        // Rotation rates.
   float rotmatrix[4][4];
   class Quaternion * qcalc;

   // Translation.
   float x, y, z;

   // Scale.
   float scale;

   // Speed.
   float speed;

   // Constructors.
   cSpacial()
   {
      float rotation[3];
      float translation[3];

      rotation[0]    = rotation[1] = rotation[2] = 0;
      translation[0] = translation[1] = translation[2] = 0.0;
      initialize(rotation, translation, 1.0, 0.0);
   }


   cSpacial(float *rotation, float *translation, float scale, float speed)
   {
      initialize(rotation, translation, scale, speed);
   }


   void initialize(float *rotation, float *translation, float scale, float speed)
   {
      float v[4];

      v[0]  = v[1] = v[2] = 0.0;
      v[3]  = 1.0;
      qcalc = new Quaternion(v);
      qcalc->build_rotmatrix(rotmatrix, qcalc->quat);
      pitch = rotation[0];
      yaw   = rotation[1];
      roll  = rotation[2];
      update();
      x           = translation[0];
      y           = translation[1];
      z           = translation[2];
      this->scale = scale;
      this->speed = speed;
   }


   // Destructor.
   ~cSpacial() { delete qcalc; }

   // Update rotation and translation state.
   void update();

   // Get direction vectors.
   void getRight(float *v)
   {
      v[0] = rotmatrix[0][0];
      v[1] = rotmatrix[0][1];
      v[2] = rotmatrix[0][2];
   }


   void getUp(float *v)
   {
      v[0] = rotmatrix[1][0];
      v[1] = rotmatrix[1][1];
      v[2] = rotmatrix[1][2];
   }


   void getForward(float *v)
   {
      v[0] = rotmatrix[2][0];
      v[1] = rotmatrix[2][1];
      v[2] = rotmatrix[2][2];
   }


   // Set direction vectors.
   void setRight(float *v)
   {
      rotmatrix[0][0] = v[0];
      rotmatrix[0][1] = v[1];
      rotmatrix[0][2] = v[2];
   }


   void setUp(float *v)
   {
      rotmatrix[1][0] = v[0];
      rotmatrix[1][1] = v[1];
      rotmatrix[1][2] = v[2];
   }


   void setForward(float *v)
   {
      rotmatrix[2][0] = v[0];
      rotmatrix[2][1] = v[1];
      rotmatrix[2][2] = v[2];
   }


   // Get model transformation matrix.
   void getModelTransform(float *matrix)
   {
      glMatrixMode(GL_MODELVIEW);
      glPushMatrix();
      glTranslatef(x, y, z);
      glMultMatrixf(&rotmatrix[0][0]);
      glScalef(scale, scale, scale);
      glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
      glPopMatrix();
   }
};

// Update rotation and translation state.
void cSpacial::update()
{
   float xq[4], yq[4], zq[4], q1[4], q2[4];
   float v[3];

   v[0] = 1.0f;
   v[1] = 0.0f;
   v[2] = 0.0f;
   qcalc->axis_to_quat(v, pitch * (M_PI / 180.0f), xq);
   v[0] = 0.0;
   v[1] = 1.0f;
   v[2] = 0.0;
   qcalc->axis_to_quat(v, roll * (M_PI / 180.0f), yq);
   qcalc->mult_quats(xq, yq, q1);
   v[0] = 0.0f;
   v[1] = 0.0f;
   v[2] = 1.0f;
   qcalc->axis_to_quat(v, yaw * (M_PI / 180.0f), zq);
   qcalc->mult_quats(q1, zq, q2);
   q1[0] = qcalc->quat[0];
   q1[1] = qcalc->quat[1];
   q1[2] = qcalc->quat[2];
   q1[3] = qcalc->quat[3];
   qcalc->mult_quats(q1, q2, qcalc->quat);
   qcalc->build_rotmatrix(rotmatrix, qcalc->quat);
   x -= (rotmatrix[1][0] * speed);
   y -= (rotmatrix[1][1] * speed);
   z -= (rotmatrix[1][2] * speed);
}


#endif                                            // #ifndef __SPACIAL_HPP__
