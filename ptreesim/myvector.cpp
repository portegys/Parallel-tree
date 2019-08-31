/* Programmer : Kevin Greenan <kmgreen2@hotmail.com>
 *  Date : 4/11/2003
 *  Filename : vector.cpp
 *  Description : Implementation of a lightweight vector class.
 *
 *  Updates:
 *
 */

#include <iostream>
#include "myvector.h"

// Scalar multiple
myVector operator*(const myVector& p_vMyVec, float p_fScalar)
{
   return(myVector(p_vMyVec.GetX() * p_fScalar,
                   p_vMyVec.GetY() * p_fScalar,
                   p_vMyVec.GetZ() * p_fScalar));
}


float operator*(const myVector& p_lVec, const myVector& p_rVec)
{
   return((p_lVec.GetX() * p_rVec.GetX()) +
          (p_lVec.GetY() * p_rVec.GetY()) +
          (p_lVec.GetZ() * p_rVec.GetZ()));
}


myVector::myVector()
{
   m_x = m_y = m_z = 0.0;
}


myVector::myVector(float p_x, float p_y, float p_z)
{
   m_x = p_x;
   m_y = p_y;
   m_z = p_z;
}


myVector::myVector(const myVector& p_vMyVec)
{
   m_x = p_vMyVec.GetX();
   m_y = p_vMyVec.GetY();
   m_z = p_vMyVec.GetZ();
}


myVector::~myVector()
{
}


float myVector::GetMagnitude() const
{
   return((float)sqrt((m_x * m_x) + (m_y * m_y) + (m_z * m_z)));
}


float myVector::GetX() const
{
   return(m_x);
}


float myVector::GetY() const
{
   return(m_y);
}


float myVector::GetZ() const
{
   return(m_z);
}


float myVector::GetDistance(const myVector& p_vMyVec) const
{
   float xDist(m_x - p_vMyVec.GetX()),
   yDist(m_y - p_vMyVec.GetY()),
   zDist(m_z - p_vMyVec.GetZ());

   xDist *= xDist;
   yDist *= yDist;
   zDist *= zDist;

   return((float)sqrt(xDist + yDist + zDist));
}


myVector myVector::GetNormalized() const
{
   float norm = this->GetMagnitude();

   return(*(this) * (norm != 0 ? float(1 / norm) : 0));
}


// Get the angle between this vector and another
float myVector::GetAngleBetween(const myVector& p_vMyVec) const
{
   return(float(acos((*(this) * p_vMyVec) /
                     (this->GetMagnitude() * p_vMyVec.GetMagnitude()))));
}


// Check to make sure this is correct, I forget???
void myVector::SetMagnitude(float p_fNewMag)
{
   myVector normalized = this->GetNormalized();

   m_x = normalized.GetX() * p_fNewMag;
   m_y = normalized.GetY() * p_fNewMag;
   m_z = normalized.GetZ() * p_fNewMag;
}


void myVector::SetX(float p_x)
{
   m_x = p_x;
}


void myVector::SetY(float p_y)
{
   m_y = p_y;
}


void myVector::SetZ(float p_z)
{
   m_z = p_z;
}


myVector myVector::operator+(const myVector& p_vMyVec)
{
   return(myVector(m_x + p_vMyVec.GetX(),
                   m_y + p_vMyVec.GetY(),
                   m_z + p_vMyVec.GetZ()));
}


myVector myVector::operator-(const myVector& p_vMyVec)
{
   return(myVector(m_x - p_vMyVec.GetX(),
                   m_y - p_vMyVec.GetY(),
                   m_z - p_vMyVec.GetZ()));
}


myVector myVector::operator+=(const myVector& p_vMyVec)
{
   return(myVector(m_x += p_vMyVec.GetX(),
                   m_y += p_vMyVec.GetY(),
                   m_z += p_vMyVec.GetZ()));
}


// The dot product
float myVector::operator*(const myVector& p_vMyVec)
{
   return(m_x * p_vMyVec.GetX() + m_y * p_vMyVec.GetY() + m_z * p_vMyVec.GetZ());
}


// Scalar Multiple
myVector myVector::operator*(float p_fScalar)
{
   return(myVector(m_x * p_fScalar, m_y * p_fScalar, m_z * p_fScalar));
}


// Assignment
myVector myVector::operator=(const myVector& p_vMyVec)
{
   m_x = p_vMyVec.GetX();
   m_y = p_vMyVec.GetY();
   m_z = p_vMyVec.GetZ();

   return(*this);
}


bool myVector::operator==(const myVector& p_vMyVec)
{
   return(m_x == p_vMyVec.GetX() &&
          m_y == p_vMyVec.GetY() &&
          m_z == p_vMyVec.GetZ() ?
          true : false);
}
