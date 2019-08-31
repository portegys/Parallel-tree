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
//* File Name: cameraGuide.hpp                                              *//
//* Author: Tom Portegys, portegys@ilstu.edu                                *//
//* Date Made: 03/25/03                                                     *//
//* File Desc: Class declaration and implementation details for base class  *//
//*            representing a camera guidance object.                       *//
//* Rev. Date:                                                              *//
//* Rev. Desc:													            *//
//*                                                                         *//
//***************************************************************************//
#ifndef __CAMERA_GUIDE_HPP__
#define __CAMERA_GUIDE_HPP__

#ifdef UNIX
#include <stdio.h>
#include <stdlib.h>
#else
#include <windows.h>
#include <cstdio>
#include <cstdlib>
#endif
#include <GL/gl.h>
#include "spacial.hpp"

class CameraGuide
{
public:

   // Constructor.
   CameraGuide()
   {
      m_spacial = new cSpacial();
      m_pitch   = m_yaw = m_roll = 0.0;
   }


   // Destructor.
   ~CameraGuide()
   {
      delete m_spacial;
   }


   // Rotations.
   GLfloat GetPitch() { return(m_pitch); }
   GLfloat GetYaw()   { return(m_yaw); }
   GLfloat GetRoll()  { return(m_roll); }
   void SetPitch(GLfloat n)
   {
      m_spacial->pitch = (n - m_pitch);
      m_spacial->update();
      m_spacial->pitch = 0.0;
      m_pitch          = n;
   }


   void SetYaw(GLfloat n)
   {
      m_spacial->yaw = (n - m_yaw);
      m_spacial->update();
      m_spacial->yaw = 0.0;
      m_yaw          = n;
   }


   void SetRoll(GLfloat n)
   {
      m_spacial->roll = (n - m_roll);
      m_spacial->update();
      m_spacial->roll = 0.0;
      m_roll          = n;
   }


   // Get direction vectors.
   void GetRight(GLfloat *v)   { m_spacial->getRight(v); }
   void GetUp(GLfloat *v)      { m_spacial->getUp(v); }
   void GetForward(GLfloat *v) { m_spacial->getForward(v); }

   // Position.
   void GetPosition(GLfloat *v)
   {
      v[0] = m_spacial->x;
      v[1] = m_spacial->y;
      v[2] = m_spacial->z;
   }


   void SetPosition(GLfloat *v)
   {
      m_spacial->x = v[0];
      m_spacial->y = v[1];
      m_spacial->z = v[2];
      m_spacial->update();
   }


   // Scale.
   GLfloat GetScale() { return(m_spacial->scale); }
   void SetScale(GLfloat n) { m_spacial->scale = n; }

   // Speed.
   GLfloat GetSpeed() { return(m_spacial->speed); }
   void SetSpeed(GLfloat n)
   {
      m_spacial->speed = n;
      if (m_spacial->speed < 0.0) { m_spacial->speed = 0.0; }
   }


   // Get model transformation matrix.
   void GetModelTransform(GLfloat *matrix)
   {
      m_spacial->getModelTransform(matrix);
   }


   // Get spacial properties.
   cSpacial *GetSpacial() { return(m_spacial); }

   // Update.
   void Update() { m_spacial->update(); }

protected:

   // Spacial properties.
   cSpacial *m_spacial;
   GLfloat  m_pitch, m_yaw, m_roll;
};
#endif                                            // #ifndef __CAMERA_GUIDE_HPP__
