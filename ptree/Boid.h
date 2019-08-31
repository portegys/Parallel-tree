/*
 * Copyright (C) 1996, Christopher John Kline
 * Electronic mail: ckline@acm.org
 *
 * This software may be freely copied, modified, and redistributed
 * for academic purposes and by not-for-profit organizations, provided that
 * this copyright notice is preserved on all copies, and that the source
 * code is included or notice is given informing the end-user that the source
 * code is publicly available under the terms described here.
 *
 * Persons or organizations wishing to use this code or any modified version
 * of this code in a commercial and/or for-profit manner must contact the
 * author via electronic mail (preferred) or other method to arrange the terms
 * of usage. These terms may be as simple as giving the author visible credit
 * in the final product.
 *
 * There is no warranty or other guarantee of fitness for this software,
 * it is provided solely "as is". Bug reports or fixes may be sent
 * to the author, who may or may not act on them as he desires.
 *
 * If you use this software the author politely requests that you inform him via
 * electronic mail.
 *
 */

// Boid.h
//
// INTERFACE: Class for objects that move with A-Life boid-type behavior.
//
// (c) 1996 Christopher Kline <ckline@acm.org>
//
// $Id: Boid.h,v 1.5 1996/07/21 17:44:45 ckline Exp ckline $

#ifndef __BOID_H
#define __BOID_H

#include <limits.h>
#include "NamedObject.h"
#include "Obstacle.h"

class Boid : public SimObject
{
public:

   enum boidTypes
   {
      NORMAL = 0,
      PREDATOR,
      PREY
   };

   Boid(Vector bPosition, Vector bVelocity, Vector bDimensions);
   // Constructor. Creates a bird of type NORMAL with the given dimensions,
   // initial position, and velocity.
   // bDimensions.{x,y,z} = {length, width, height}

   Boid(Vector bPosition, Vector bVelocity, Vector bDimensions,
        int bBoidType, int bBoidNumber);
   // Construct specified boid.

   Boid *clone();

   // Clone boid.

   virtual void aim(Boid *boidList);

   // Updates velocity and acceleration and determines the new position of
   // the object based on the previous acceleration and velocity.

   virtual void move(void);

   // Move the object to the position determined by aim().

   double maxVelocity;
   // [m/sec] Maximum magnitude of velocity. Default value is 10

   double maxAcceleration;
   // [m/(sec^2)] Maximum magnitude of acceleration as a fraction of
   // maxVelocity. Default value is 0.5

   double cruiseDistance;
   // [m] Desired distance from closest neighbor when flying. Default value
   // is twice the bodylength.

   double roll;
   // [radians] Rotation around body-local z-axis (+roll =
   // counterclockwise). Default value is 0.

   double pitch;
   // [radians] Rotation around body-local x-axis (+pitch = nose tilting
   // upward). Default value is 0.

   double yaw;
   // [radians] Rotation around body-local y-axis (increasing
   // counterclockwise, 0 is along body-local +z). Default value is 0.

   Boid *next;
   // For linking boids.

   static ObstacleList obstacles;
   // Linked list of obstacles that every boid will try to avoid. By default
   // there are no obstacles. If you add an obstacle to this list, _all_
   // boids will see it (because it's a static member).

   static double visibilityRange;
   static void setVisibilityRange(double range) { visibilityRange = range; }
   // Boid visibility range.

   static double span;
   static void setSpan(double s) { span = s; }
   // Distance from origin to XYZ limit of navigable space.

   static double margin;
   static void setMargin(double m) { margin = m; }
   // Defines a buffer zone before the XYZ limits.

   static double updateRate;
   static void setUpdateRate(double r) { updateRate = r; }
   // Update rate of the boids (internally) in frames-per-second.

   static void setBoidCount(int c) { boidCount = c; }
   // Set the boid id counter.

   virtual int getBoidType(void) const;

   // Returns the type of this boid.

   int getBoidNumber() { return(boidNumber); }
   // Returns the number of this boid.

protected:

   virtual double getGravAcceleration(void) const;

   // Returns the magnitude of gravitational acceleration in the (0, -1, 0)
   // direction [m/(sec^2)].

   virtual double accumulate(Vector& accumulator, Vector valueToAdd);

   // Given an accumulator and a value to add to the accumulator, this
   // method truncates the magnitude of valueToAdd so that, when added to the
   // accumulator, the magnitude of the accumulator is at most 1.0. It then
   // adds the truncated value to the accumulator. The value returned is the
   // magnitude of the accumulator after the addition.

   virtual float getProbeLength(void);

   // Returns how far in front of boid to probe for obstacles. By default,
   // the probe length scales linearly from 10 times bodylength to 50 times
   // bodylength as the boid accelerates from 0 m/s to maxVelocity.

   virtual double desiredCruisingSpeed(void);

   // Returns the speed the boid would like to travel at when not under any
   // other influences (i.e., obstacles, flocking desires, etc). The default
   // value is 1/5 of maxVelocity.

   virtual void calculateRollPitchYaw(Vector appliedAcceleration,
                                      Vector currentVelocity,
                                      Vector currentPosition);

   // Calculate the roll, pitch, and yaw of this boid based on its
   // acceleration, velocity, and position. Though position isn't necessary
   // for most approximations of attitude, it may be useful in some
   // circumstances.

   virtual bool visibleToSelf(Boid *b);

   // Returns true if this boid can see boid b.

   virtual Vector levelFlight(Vector AccelSoFar);

   // Returns a vector which indicates how the boid would like to accelerate
   // in order to fly level (i.e., with minimal pitch).

   virtual Vector wander(void);

   // Returns a vector which indicates how the boid would like to accelerate
   // when not under any other influences. Related to desiredCruisingSpeed().

   virtual Vector collisionAvoidance(void);

   // Returns a vector which indicates how the boid would like to accelerate
   // in order to avoid collisions with non-boid obstacles.

   virtual Vector resolveCollision(Vector pointOnObject, Vector normalToObject);

   // Called by CollisionAvoidance, this method attempts to avoid a collision
   // with a specific obstacle, and returns an acceleration vector indicating
   // how the boid should accelerate to achieve this end.

   virtual Vector maintainingCruisingDistance(Boid *boidList);

   // Returns a vector which indicates how the boid would like to accelerate
   // in order to maintain a distance of cruiseDistance from the nearest
   // visible boid.

   virtual Vector velocityMatching(Boid *boidList);

   // Returns a vector which indicates how the boid would like to accelerate
   // in order to fly at approximately the same speed and direction as the
   // nearby boids.

   virtual Vector flockCentering(Boid *boidList);

   // Returns a vector which indicates how the boid would like to accelerate
   // in order to be near the center of the flock.

   virtual Vector navigator(Boid *boidList);

   // This method prioritizes and resolves the acceleration vectors from
   // CollisionAvoidance(), FlockCentering(), MaintainingCruisingDistance(),
   // VelocityMatching(), Wander(), and LevelFlight(). It returns the actual
   // acceleration vector that the boid will apply to its flight in the
   // current time step. It is supplied a list of visible boids.

   double bodyLength;
   // [m] Length of the boid. By default this value is equal to the z
   // component of the bDimensions passed to the constructor.

   double time;
   // [sec] current time

   int boidNumber;
   // Unique integer identifying the number of this boid. The first boid
   // created is given boidNumber 1, and the values increase sequentially.

   char bName[50];
   // Name of self. This is used by the ObjectList data structure. The
   // constructor sets this equal to "boidX" where X is the boidNumber.

   bool flockSelectively;
   // Should this boid flock only with boids of the same boidType, or with
   // all boids? The default value is false, meaning that this boid will
   // flock with all boids regardless of type.
   // Basically, should boids of a feather stick together? :)

   int boidType;
   // Identifies the type of boid for selective flocking

   static int boidCount;
   // Total number of boids that have been created so far. Note that this
   // value increases monotonically, and does NOT take into account boids
   // that have been deleted.

private:

   Vector newPosition;
   // New position.

   Vector oldVelocity;
   // [m/sec] velocity at last update.

   Vector acceleration;
   // [m/(sec^2)] acceleration requested at last update.

   bool flightflag;
   // Has the boid been updated at least once?
};

// ------------------------------------------------ inline methods ------------------------------------------------

inline double
Boid::getGravAcceleration(void) const
{
   return(9.806650);
}


inline int
Boid::getBoidType(void) const
{
   return(boidType);
}


inline bool
Boid::visibleToSelf(Boid *b)
{
   // find out if the boid b is within our field of view
   Vector vectorToObject = b->position - position;

   // This isn't perfectly accurate, since we're not always facing in
   // the direction of our velocity, but it's close enough.
   // pi/3 radians is our FOV
   if (AngleBetween(velocity, vectorToObject) <= 1.0471967)
   {
      return(true);
   }
   else
   {
      return(false);
   }
}


inline float
Boid::getProbeLength(void)
{
   float maxScale = 5;

   // When we're at maxVelocity, scalefactor = maxScale.
   // When our velocity is 0, scalefactor = 1.
   // Linearly scale in between.
   float scaleFactor = ((maxScale - 1) / maxVelocity) * Magnitude(velocity) + 1;

   return(10 * bodyLength * scaleFactor);
}


inline double
Boid::desiredCruisingSpeed(void)
{
   return(0.2 * maxVelocity);
}


inline Vector
Boid::levelFlight(Vector AccelSoFar)
{
   // Determine the vertical acceleration.
   Vector verticalAcc(0, AccelSoFar.y, 0);

   // Try to negate it.
   return(-verticalAcc);
}


#endif                                            /* __BOID_H */
