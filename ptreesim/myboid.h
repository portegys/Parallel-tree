/* Programmer : Kevin Greenan <kmgreen2@hotmail.com>
 *  Date : 4/11/2003
 *  Filename : myboid.h
 *  Description : A lightwieght and simplified boid object class
 *                very similar to that of Christopher Kline <ckline@acm.org>.
 *
 *  Updates :
 *
 */

#ifndef __MYBOID_H__
#define __MYBOID_H__

#include "myvector.h"
#include <list>

typedef enum {
   LOCAL, REMOTE
}
location_t;

class Boid
{
public:
   Boid();
   Boid(const myVector& p_vVel, const myVector& p_vPos, const myVector& p_vDim,
        const double p_dTime, int p_iId);
   myVector GetVelocity() const;
   myVector GetPosition() const;
   myVector GetDimension() const;
   float GetMaxVelocity() const;
   float GetMaxAcceleration() const;
   float GetDesiredDist() const;
   float GetDesiredSpeed() const;
   myVector GetOldVelocity() const;
   myVector GetAcceleration() const;
   int GetBoidNum() const;
   bool HasUpdated() const;
   bool Update(std::list<Boid>& p_lVisList, double p_dTime);
   void Move();

   location_t   m_ltLocation;
   static float SPAN;
   static float MARGIN;
   static float TOLERANCE;
   static float SEPARATEFACTOR;
   static float APPROACHFACTOR;
   static float MATCHVELFACTOR;
   static float FLOCKCENFACTOR;
   static float AVOIDCOLLISIONFACTOR;
   static float MAXADJUSTEDACC;
   static float MAXACCELERATION;
   static float MAXVELOCITY;
   static float DESIREDDIST;
   static float DESIREDSPEED;
   static float VISIBILITYRANGE;

   static void setSpan(float s) { SPAN = s; }
   static void setMargin(float m) { MARGIN = m; }

private:

   myVector Navigate(const std::list<Boid>& p_lVisList);
   float Accumulate(myVector& p_vAccumulator, const myVector& p_vAddedValue);
   myVector Wander();
   myVector AvoidCollision(const std::list<Boid>& p_lVisList) const;
   myVector MaintainDesiredDist(const std::list<Boid>& p_lVisList);
   myVector MatchVelocity(const std::list<Boid>& p_lVisList);
   myVector CenterOfFlock(const std::list<Boid>& p_lVisList);

   myVector m_vVelocity;                          // Velocity of the object
   myVector m_vPosition;                          // Pos of center of object
   myVector m_vDimension;                         // Dim of the object (x,y,z)
   float    m_fMaxVelocity;
   float    m_fMaxAcceleration;
   float    m_fDesiredDist;
   float    m_fDesiredSpeed;
   myVector m_vOldVelocity;
   myVector m_vNewPosition;
   myVector m_vAcceleration;
   bool     m_bUpdateFlag;
   int      m_iBoidNum;
   double   m_dCurrTime;
};
#endif
