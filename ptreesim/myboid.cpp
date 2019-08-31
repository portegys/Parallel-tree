/* Programmer : Kevin Greenan <kmgreen2@hotmail.com>
 *  Date : 4/11/2003
 *  Filename : myboid.h
 *  Description : A lightweight and simplified boid object class
 *                very similar to that of Christopher Kline <ckline@acm.org>.
 *
 *  Updates :
 *
 */

#include "myboid.h"

#ifndef FLT_MAX
const float FLT_MAX = 100000;
#endif

// Space size.
float Boid::SPAN   = float(1.0);
float Boid::MARGIN = float(0.5);

// Values we can "tweak"
float Boid::TOLERANCE            = 1.0;           // Working Test Values
float Boid::SEPARATEFACTOR       = float(0.09);   // 0.09
float Boid::APPROACHFACTOR       = float(0.05);   // 0.05
float Boid::MATCHVELFACTOR       = float(0.05);   // 0.05
float Boid::FLOCKCENFACTOR       = float(0.15);   // 0.15
float Boid::AVOIDCOLLISIONFACTOR = float(0.75);   // 0.75
float Boid::VISIBILITYRANGE      = float(5.0);
// Because it is a percentage of MaxAcceleration
float Boid::MAXADJUSTEDACC  = float(1.0);         // 1.0
float Boid::MAXACCELERATION = float(0.5);         // 0.5
float Boid::MAXVELOCITY     = float(0.1);         // 1.0
float Boid::DESIREDDIST     = float(5.0);         // 5.0
float Boid::DESIREDSPEED    = float(0.5);         // 0.5

// Default...use for remote boid construction
Boid::Boid()
{
   m_fMaxVelocity     = MAXVELOCITY;
   m_fMaxAcceleration = MAXACCELERATION;
   m_fDesiredDist     = DESIREDDIST;
   m_fDesiredSpeed    = DESIREDSPEED;
   m_bUpdateFlag      = false;
   m_dCurrTime        = 0.0;
   m_ltLocation       = REMOTE;
}


Boid::Boid(const myVector& p_vVel, const myVector& p_vPos, const myVector& p_vDim,
           const double p_dTime, int p_iId)
{
   m_vVelocity        = m_vOldVelocity = p_vVel;
   m_vPosition        = m_vNewPosition = p_vPos;
   m_vDimension       = p_vDim;
   m_fMaxVelocity     = MAXVELOCITY;
   m_fMaxAcceleration = MAXACCELERATION;
   m_fDesiredDist     = DESIREDDIST;
   m_fDesiredSpeed    = DESIREDSPEED;
   m_bUpdateFlag      = false;
   m_iBoidNum         = p_iId;
   m_dCurrTime        = p_dTime;
   m_ltLocation       = LOCAL;
}


myVector Boid::GetVelocity() const
{
   if (m_ltLocation == LOCAL)
   {
      return(m_vVelocity);
   }
   else
   {
      // Get remotely
      // return myVector();
   }
}


myVector Boid::GetPosition() const
{
   if (m_ltLocation == LOCAL)
   {
      return(m_vPosition);
   }
   else
   {
      // Get remotely
      // return myVector();
   }
}


myVector Boid::GetDimension() const
{
   return(m_vDimension);
}


float Boid::GetMaxVelocity() const
{
   return(m_fMaxVelocity);
}


float Boid::GetMaxAcceleration() const
{
   return(m_fMaxAcceleration);
}


float Boid::GetDesiredDist() const
{
   return(m_fDesiredDist);
}


float Boid::GetDesiredSpeed() const
{
   return(m_fDesiredSpeed);
}


myVector Boid::GetOldVelocity() const
{
   return(m_vOldVelocity);
}


myVector Boid::GetAcceleration() const
{
   if (m_ltLocation == LOCAL)
   {
      return(m_vAcceleration);
   }
   else
   {
      // Get remotely
      // return myVector();
   }
}


int Boid::GetBoidNum() const
{
   return(m_iBoidNum);
}


bool Boid::HasUpdated() const
{
   return(m_bUpdateFlag);
}


bool Boid::Update(std::list<Boid>& p_lVisList, double p_dTime)
{
   if (m_bUpdateFlag == false)
   {
      m_bUpdateFlag  = true;
      m_vNewPosition = m_vPosition;
      m_dCurrTime    = p_dTime;
   }
   else
   {
      float timeDiff = float(p_dTime - m_dCurrTime);
      // New current time
      m_dCurrTime = p_dTime;

      m_vNewPosition = m_vPosition + ((m_vOldVelocity * timeDiff) +
                                      (m_vAcceleration * float(0.5) * timeDiff * timeDiff));

      m_vVelocity += (m_vAcceleration * timeDiff);

      if (m_vVelocity.GetMagnitude() > m_fMaxVelocity)
      {
         m_vVelocity.SetMagnitude(m_fMaxVelocity);
      }

      // Reverse direction if reach limits of space.
      if (m_vNewPosition.GetX() < -SPAN + MARGIN)
      {
         if (m_vVelocity.GetX() < 0.0)
         {
            m_vVelocity.SetX(-m_vVelocity.GetX());
         }
      }
      if (m_vNewPosition.GetX() > SPAN - MARGIN)
      {
         if (m_vVelocity.GetX() > 0.0)
         {
            m_vVelocity.SetX(-m_vVelocity.GetX());
         }
      }
      if (m_vNewPosition.GetY() < -SPAN + MARGIN)
      {
         if (m_vVelocity.GetY() < 0.0)
         {
            m_vVelocity.SetY(-m_vVelocity.GetY());
         }
      }
      if (m_vNewPosition.GetY() > SPAN - MARGIN)
      {
         if (m_vVelocity.GetY() > 0.0)
         {
            m_vVelocity.SetY(-m_vVelocity.GetY());
         }
      }
      if (m_vNewPosition.GetZ() < -SPAN + MARGIN)
      {
         if (m_vVelocity.GetZ() < 0.0)
         {
            m_vVelocity.SetZ(-m_vVelocity.GetZ());
         }
      }
      if (m_vNewPosition.GetZ() > SPAN - MARGIN)
      {
         if (m_vVelocity.GetZ() > 0.0)
         {
            m_vVelocity.SetZ(-m_vVelocity.GetZ());
         }
      }

      m_vOldVelocity = m_vVelocity;

      m_vAcceleration = Navigate(p_lVisList);

      if (m_vAcceleration.GetMagnitude() > m_fMaxAcceleration)
      {
         m_vAcceleration.SetMagnitude(m_fMaxAcceleration);
      }
   }
   return(true);
}


void Boid::Move()
{
   m_vPosition = m_vNewPosition;
}


// The way the boid acts when not under any outside
// influence.  Taken straight from Christopher Klien's code
myVector Boid::Wander()
{
   double dist =
      (m_vVelocity.GetMagnitude() - GetDesiredSpeed()) / GetMaxVelocity();

   double urgency = fabs(dist);

   if (urgency > 0.25)
   {
      urgency = 0.25;
   }

   return(m_vVelocity.GetNormalized() * float(urgency * (dist > 0 ? -1.0 : 1.0)));
}


// Check all of the objects on the "near" list.
// Return an acceleration which will avoid a
// potential collision with closest boid.
// Velocity is used to reflect possible future position.
// This is done because these computations will be done in parallel.
myVector Boid::AvoidCollision(const std::list<Boid>& p_lVisList) const
{
   std::list<Boid>::const_iterator listItr;
   Boid     tempBoid;
   myVector accAdjust, closestVel;
   float    tempDist(0), closestDist(FLT_MAX);
   listItr = p_lVisList.begin();

   while (listItr != p_lVisList.end())
   {
      tempBoid = *listItr;
      listItr++;

      // Do not check yourself
      if (tempBoid.GetBoidNum() == this->m_iBoidNum)
      {
         continue;
      }

      // If others are outside of tolerance, then do not adjust
      if (m_vOldVelocity.GetDistance(tempBoid.GetVelocity()) > Boid::TOLERANCE)
      {
         continue;
      }
      // If others are within the tolerance, must avoid a
      // potential collision.
      // Find the closest "boid" to me, get its velocity
      tempDist = m_vPosition.GetDistance(tempBoid.GetPosition());
      if (tempDist < closestDist)
      {
         closestDist = tempDist;
         closestVel  = tempBoid.GetVelocity();
      }
   }
   // Change my acceleration such that I will follow the velocity of the approaching boid,
   // if it is approaching.
   if (closestDist > m_vPosition.GetDistance(tempBoid.GetPosition() + (tempBoid.GetVelocity() * 1.0)))
   {
      accAdjust = tempBoid.GetVelocity() * Boid::AVOIDCOLLISIONFACTOR;
      accAdjust = accAdjust.GetNormalized();
      return(accAdjust);
   }
   return(accAdjust);
}


// Make sure the boid maintains the desired dist, but
// stays close. Based on Christopher Klien's code.
// Old velocity is used to reflect possible future position.
// This is done because these computations will be done in parallel.
// Might be a good idea to think of something else.
myVector Boid::MaintainDesiredDist(const std::list<Boid>& p_lVisList)
{
   std::list<Boid>::const_iterator listItr;
   Boid     tempBoid;
   float    distClosestNeighbor(FLT_MAX), tempDist;
   float    adjustX(0.0), adjustY(0.0), adjustZ(0.0);
   myVector separation;
   listItr = p_lVisList.begin();

   while (listItr != p_lVisList.end())
   {
      tempBoid = *listItr;
      listItr++;

      // Do not check yourself
      if (tempBoid.GetBoidNum() == this->m_iBoidNum)
      {
         continue;
      }

      // If the distance between this and other boid is
      // greater than desired distance then ignore.
      if ((tempDist = m_vPosition.GetDistance(tempBoid.GetPosition()) > m_fDesiredDist))
      {
         continue;
      }

      // Determine the closest neighbor, stay away, but
      // close as possible.
      if (tempDist < distClosestNeighbor)
      {
         distClosestNeighbor = tempDist;
      }
   }

   // Adjust the acceleration to stay close, but not too close
   if (distClosestNeighbor != FLT_MAX)
   {
      separation = m_vPosition - tempBoid.GetPosition();

      if (separation.GetX() < m_fDesiredDist)
      {
         adjustX -= Boid::SEPARATEFACTOR;
      }
      else if (separation.GetX() > m_fDesiredDist)
      {
         adjustX += Boid::APPROACHFACTOR;
      }
      if (separation.GetY() < m_fDesiredDist)
      {
         adjustY -= Boid::SEPARATEFACTOR;
      }
      else if (separation.GetY() > m_fDesiredDist)
      {
         adjustY += Boid::APPROACHFACTOR;
      }
      if (separation.GetZ() < m_fDesiredDist)
      {
         adjustZ -= Boid::SEPARATEFACTOR;
      }
      else if (separation.GetZ() > m_fDesiredDist)
      {
         adjustZ += Boid::APPROACHFACTOR;
      }
   }

   return(myVector(adjustX, adjustY, adjustZ));
}


// Old velocity is used to reflect possible future position.
// This is done because these computations will be done in parallel.
// Might be a good idea to think of something else.
myVector Boid::MatchVelocity(const std::list<Boid>& p_lVisList)
{
   myVector velClosestNeighbor;
   float    tempDist(0), distClosestNeighbor(FLT_MAX);

   std::list<Boid>::const_iterator listItr;
   Boid tempBoid;

   listItr = p_lVisList.begin();

   while (listItr != p_lVisList.end())
   {
      tempBoid = *listItr;
      listItr++;

      // Do not check yourself
      if (tempBoid.GetBoidNum() == this->m_iBoidNum)
      {
         continue;
      }

      tempDist = m_vPosition.GetDistance(tempBoid.GetPosition());

      if (tempDist < distClosestNeighbor)
      {
         distClosestNeighbor = tempDist;
         velClosestNeighbor  = tempBoid.GetVelocity();
      }
   }

   if (distClosestNeighbor != FLT_MAX)
   {
      // Not sure why this is 0.05
      // Wait, I think it reflects the
      // percent of overall acceleration
      // to be affected.
      velClosestNeighbor.SetMagnitude(MATCHVELFACTOR);
   }

   return(velClosestNeighbor);
}


myVector Boid::CenterOfFlock(const std::list<Boid>& p_lVisList)
{
   myVector centerOfFlock, accelToCenter;
   int      numBoids(0);

   std::list<Boid>::const_iterator listItr;
   Boid tempBoid;

   listItr = p_lVisList.begin();

   while (listItr != p_lVisList.end())
   {
      tempBoid = *listItr;
      listItr++;

      centerOfFlock += tempBoid.GetPosition();
      numBoids++;
   }

   if (numBoids)
   {
      centerOfFlock = (centerOfFlock * float(1 / numBoids));
      // Should try to move the flock
      accelToCenter = centerOfFlock - m_vPosition;
      accelToCenter.SetMagnitude(FLOCKCENFACTOR);
   }
   return(accelToCenter);
}


// Accumulates the acceleration values and returns a current magnitude.
float Boid::Accumulate(myVector& p_vAccumulator, const myVector& p_vAddedValue)
{
   float magLeft    = float(Boid::MAXADJUSTEDACC - p_vAccumulator.GetMagnitude());
   float magOfValue = p_vAddedValue.GetMagnitude();
   float newMag     = magOfValue < magLeft ? magOfValue : magLeft;

   p_vAccumulator += (p_vAddedValue.GetNormalized() * newMag);
   return(newMag);
}


// Goes down the acceleration adjusting pipeline.
myVector Boid::Navigate(const std::list<Boid>& p_lVisList)
{
   myVector Accumulator;

   if (Accumulate(Accumulator, CenterOfFlock(p_lVisList)) >= MAXADJUSTEDACC)
   {
   }
   else if (Accumulate(Accumulator, MaintainDesiredDist(p_lVisList)) >= MAXADJUSTEDACC)
   {
   }
   else if (Accumulate(Accumulator, AvoidCollision(p_lVisList)) >= MAXADJUSTEDACC)
   {
   }
   else if (Accumulate(Accumulator, MatchVelocity(p_lVisList)) >= MAXADJUSTEDACC)
   {
   }
   else if (Accumulate(Accumulator, Wander()) >= MAXADJUSTEDACC)
   {
   }

   Accumulator = Accumulator * m_fMaxVelocity * m_fMaxAcceleration;
   return(Accumulator);
}
