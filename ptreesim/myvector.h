/* Programmer : Kevin Greenan <kmgreen2@hotmail.com>
 *  Date : 4/11/2003
 *  Filename : vector.h
 *  Description : A lightweight vector class.
 *
 *  Updates:
 *
 */

#ifndef __MYVECTOR_H__
#define __MYVECTOR_H__

#include <cmath>

class myVector
{
public:
   myVector();
   myVector(float p_x, float p_y, float p_z);
   myVector(const myVector& p_vMyVec);
   ~myVector();

   // Some accessor functions
   float GetMagnitude() const;
   float GetX() const;
   float GetY() const;
   float GetZ() const;
   float GetDistance(const myVector& p_vMyVec) const;
   myVector GetNormalized() const;
   float GetAngleBetween(const myVector& p_vMyVec) const;

   // Vector Operations
   void SetMagnitude(float p_fNewMag);
   void SetX(float p_x);
   void SetY(float p_y);
   void SetZ(float p_z);
   myVector operator+(const myVector& p_vMyVec);
   myVector operator-(const myVector& p_vMyVec);
   myVector operator+=(const myVector& p_vMyVec);

   // Dot product
   float operator*(const myVector& p_vMyVec);
   myVector operator*(float p_fScalar);
   myVector operator=(const myVector& p_vMyVec);
   bool operator==(const myVector& p_vMyVec);

private:
   float m_x, m_y, m_z;
};

// Some non-member overloaded ops
// Scalar multiple
myVector operator*(const myVector& p_vMyVec, float p_fScalar);

// Dot product
float operator*(const myVector& p_lVec, const myVector& p_rVec);
#endif
