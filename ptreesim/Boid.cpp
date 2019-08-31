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

// Boid.c++
//
// CODE: Class for objects that move with A-Life boid-type behavior.
//
// (c) 1996 Christopher Kline <ckline@acm.org>
//

#ifndef __BOID_C
#define __BOID_C

#include "Boid.h"
#include <stdio.h>
#include <assert.h>

#define M_PI       3.14159265358979323846
#define DBL_MAX    1.7976931348623157E+308

// -- Static member variables
ObstacleList Boid::obstacles;
int Boid::         boidCount       = 0;
double Boid::      visibilityRange = VISIBILITY_RANGE;
double Boid::      span            = 15.0;
double Boid::      updateRate      = 0.5;
// --
double
Boid::accumulate(Vector& accumulator, Vector valueToAdd)
{
   double magnitudeRemaining = 1.0 - Magnitude(accumulator);
   double vMag   = Magnitude(valueToAdd);
   double newMag = vMag < magnitudeRemaining ? vMag : magnitudeRemaining;

   accumulator += (Direction(valueToAdd) * newMag);

   return(newMag);
}


Vector
Boid::navigator(const std::list<Boid>& boidList)
{
   Vector vacc(0, 0, 0);                          // vector accumulator

   if (accumulate(vacc, collisionAvoidance()) >= 1.0)
   {
      goto MAXACCEL_ATTAINED;
   }
   if (accumulate(vacc, flockCentering(boidList)) >= 1.0)
   {
      goto MAXACCEL_ATTAINED;
   }
#ifdef NEVER
   // This causes a problem when boids are not run sequentially.
   if (accumulate(vacc, maintainingCruisingDistance(boidList)) >= 1.0)
   {
      goto MAXACCEL_ATTAINED;
   }
#endif
   if (accumulate(vacc, velocityMatching(boidList)) >= 1.0)
   {
      goto MAXACCEL_ATTAINED;
   }
   if (accumulate(vacc, wander()) >= 1.0)
   {
      goto MAXACCEL_ATTAINED;
   }
   if (accumulate(vacc, levelFlight(vacc)) >= 1.0)
   {
      goto MAXACCEL_ATTAINED;
   }

MAXACCEL_ATTAINED:                                /* label */

   // IMPORTANT:
   // Since the FlockCentering, CollisionAvoidance, and VelocityMatching modules
   // return a vector whose magnitude is a percentage of the maximum acceleration,
   // we need to convert this to a real magnitude before we hand it off to the Flight
   // module.
   // Remember, maxAcceleration is in terms of a fraction (0.0 to 1.0) of maxVelocity

   vacc *= maxAcceleration * maxVelocity;
   return(vacc);
}


void
Boid::calculateRollPitchYaw(Vector appliedAcceleration,
                            Vector currentVelocity,
                            Vector /*currentPosition*/)
{
// NOTES:
//
// 1) currentPosition (the third argument) is unused, though in a more
// advanced flight model it could make a difference (some flight dynamics
// change depending on altitude, for example)
//
// 2) Dr. Craig Wanke of the Mitre Corporation in McLean, Virginia helped me
// tremendously when working out these equations. Thanks, Craig!

// In our simple flight model, roll only a function of lateral
// (centripedal) acceleration and gravitational acceleration. We assume
// that gravitational acceleration will NOT be zero.

// Determine the direction of the lateral acceleration.
   Vector lateralDir = Direction((currentVelocity % appliedAcceleration) % currentVelocity);

// Set the lateral acceleration's magnitude. The magnitude is the vector
// projection of the appliedAcceleration vector onto the direction of the
// lateral acceleration).
   double lateralMag = appliedAcceleration * lateralDir;

   if (lateralMag == 0)
   {
      roll = 0;
   }
   else
   {
      roll = -atan2(getGravAcceleration(), lateralMag) + M_PI / 2;
   }

   pitch = -atan(currentVelocity.y /
                 sqrt(currentVelocity.z * currentVelocity.z
                      + currentVelocity.x * currentVelocity.x));

   yaw = atan2(currentVelocity.x, currentVelocity.z);
}


Vector
Boid::wander(void)
{
   double distanceFromCruiseSpeed =
      (Magnitude(velocity) - desiredCruisingSpeed()) / maxVelocity;

   double urgency = fabs(distanceFromCruiseSpeed);

   if (urgency > 0.25)
   {
      urgency = 0.25;
   }

   return(Direction(velocity) * urgency * (distanceFromCruiseSpeed > 0 ? -1 : 1));
}


Vector
Boid::collisionAvoidance(void)
{
   Obstacle  *obs;
   ISectData d;
   Vector    normalToObject(0, 0, 0);
   int       objectSeen = 0;
   Vector    pointOnObject;

   // Ignore obstacles that are out of the range of our probe.
   double distanceToObject = getProbeLength();

   // Find closest imminent collision with non-boid object
   obstacles.ResetIter();
   while ((obs = obstacles.Iter()) != NULL)
   {
      d = obs->DoesRayIntersect(Direction(velocity), position);

      if (d.intersectionflag == 1)
      {
         // Velocity vector intersects an obstacle

         if (Magnitude(d.point - position) <= distanceToObject)
         {
            // found a closer object
            objectSeen       = 1;
            distanceToObject = Magnitude(d.point - position);
            normalToObject   = d.normal;
            pointOnObject    = d.point;
         }
      }
   }

   if (!objectSeen)
   {
      return(Vector(0, 0, 0));
   }

   return(resolveCollision(pointOnObject, normalToObject));
}


Vector
Boid::resolveCollision(Vector pointOnObject, Vector normalToObject)
{
   double distanceToObject = Magnitude(pointOnObject - position);

   // Make sure the object's normal is pointing towards the boid.
   // The boid wants to head in the direction of the normal, which
   // should push it away from the obstacle if the normal is pointing
   // towards the boid.
   if (AngleBetween(velocity, normalToObject) < M_PI / 2)
   {
      normalToObject = -normalToObject;
   }

   double mag = 1 - distanceToObject / getProbeLength();

   // Ignore objects that are farther away than our probe.
   if (mag < 0)
   {
      mag = 0;
   }

   return(Direction(normalToObject) * mag);
}


Vector
Boid::maintainingCruisingDistance(const std::list<Boid>& boidList)
{
   double distanceToClosestNeighbor = DBL_MAX;    // DBL_MAX defined in <limits.h>
   int    foundClosestNeighbor      = 0;
   Boid   *closestNeighbor;
   double tempDistance;

   std::list<Boid>::const_iterator listItr;

   // Cycle through visible boids.
   for (listItr = boidList.begin(); listItr != boidList.end(); listItr++)
   {
      Boid n = *listItr;

      // Skip boids that we don't need to consider
      if (!visibleToSelf(&n))
      {
         continue;
      }
      if ((n.boidType != boidType) && flockSelectively)
      {
         continue;
      }

      // Find distance from the current boid to self
      tempDistance = Magnitude(n.position - position);

      // remember distance to closest boid
      if (tempDistance < distanceToClosestNeighbor)
      {
         distanceToClosestNeighbor = tempDistance;
         foundClosestNeighbor      = 1;
         closestNeighbor           = &n;
      }
   }

   Vector speedAdjustmentVector(0, 0, 0);

   if (foundClosestNeighbor)
   {
      // Have the boid try to remain at least cruiseDistance away from its
      // nearest neighbor at all times in all directions (i.e., don't violate
      // your neighbor's "personal space" bounding sphere of radius
      // cruiseDistance, but stay as close to the neighbor as possible).

      Vector separationVector = closestNeighbor->getPosition() - position;

      float separateFactor = 0.09;
      float approachFactor = 0.05;

      if (separationVector.y < cruiseDistance)
      {
         speedAdjustmentVector.y -= separateFactor;
      }
      else if (separationVector.y > cruiseDistance)
      {
         speedAdjustmentVector.y += approachFactor;
      }

      if (separationVector.x < cruiseDistance)
      {
         speedAdjustmentVector.x -= separateFactor;
      }
      else if (separationVector.x > cruiseDistance)
      {
         speedAdjustmentVector.x += approachFactor;
      }

      if (separationVector.z < cruiseDistance)
      {
         speedAdjustmentVector.z -= separateFactor;
      }
      else if (separationVector.z > cruiseDistance)
      {
         speedAdjustmentVector.z += approachFactor;
      }
   }
   // -- Otherwise, if we couldn't find a closest boid, speedAdjustmentVector
   //    will have a magnitude of 0 and thus have no effect on navigation.

   return(speedAdjustmentVector);
}


Vector
Boid::velocityMatching(const std::list<Boid>& boidList)
{
   Vector velocityOfClosestNeighbor(0, 0, 0);
   double tempDistance;
   double distanceToClosestNeighbor = DBL_MAX;

   std::list<Boid>::const_iterator listItr;

   // Cycle through visible boids.
   for (listItr = boidList.begin(); listItr != boidList.end(); listItr++)
   {
      Boid n = *listItr;

      // Skip boids that we don't need to consider
      if (!visibleToSelf(&n))
      {
         continue;
      }
      if ((n.boidType != boidType) && flockSelectively)
      {
         continue;
      }

      // Find distance from the current boid to self
      tempDistance = Magnitude(n.position - position);

      // remember velocity vector of closest boid
      if (tempDistance < distanceToClosestNeighbor)
      {
         distanceToClosestNeighbor = tempDistance;
         velocityOfClosestNeighbor = n.velocity;
      }
   }

   // If we found a close boid, set the percentage of our acceleration that
   // we want to use in order to begin flying parallel to its velocity vector.
   // -- Otherwise, if we couldn't find a closest boid, velocityOfClosestNeighbor
   //    will have a magnitude of 0 and thus have no effect on navigation.
   if (distanceToClosestNeighbor != DBL_MAX)
   {
      // return velocity vector of closest boid so we can try to match it
      velocityOfClosestNeighbor.SetMagnitude(0.05);
   }

   return(velocityOfClosestNeighbor);
}


Vector
Boid::flockCentering(const std::list<Boid>& boidList)
{
   Vector t;
   double boids_observed = 0;                     // number of boids that were checked
   Vector flockcenter(0, 0, 0);                   // approximate center of flock

   std::list<Boid>::const_iterator listItr;

   // Calculate approximate center of flock by averaging the positions of all
   // visible boids that we are flocking with.

   // Cycle through visible boids.
   for (listItr = boidList.begin(); listItr != boidList.end(); listItr++)
   {
      Boid n = *listItr;

      if (!visibleToSelf(&n))
      {
         continue;
      }
      if ((n.boidType != boidType) && flockSelectively)
      {
         continue;
      }

      flockcenter += n.position;
      boids_observed++;
   }

   if (boids_observed != 0)
   {
      flockcenter /= boids_observed;

      // now calculate a vector to head towards center of flock
      t = flockcenter - position;

      // and the percentage of maximum acceleration (in decimal) to use when yaw toward center
      t.SetMagnitude(0.1);
   }
   else
   {
      // Don't see any other birds.
      t.SetMagnitude(0);
   }

   return(t);
}


// Update velocity and acceleration, and determine new position.
void
Boid::aim(const std::list<Boid>& boidList, float simRate)
{
   if (flightflag == false)
   {
      flightflag  = true;
      newPosition = position;
   }
   else
   {
      double dt = Boid::updateRate * simRate;

      // Step 1: Calculate new position and velocity by integrating
      //         the acceleration vector.

      // Update position based on where we wanted to go, and how long it has
      // been since we made the decision to go there
      newPosition = position + (oldVelocity * dt + 0.5 * acceleration * dt * dt);

      // Set new velocity, which will be in the direction boid is traveling.
      velocity += (acceleration * dt);

      // Cap off velocity at maximum allowed value
      if (Magnitude(velocity) > maxVelocity)
      {
         velocity.SetMagnitude(maxVelocity);
      }

      // Reverse direction if reach limits of space.
      if (newPosition.x <= -span)
      {
         newPosition.x = -(span - 0.0001);
         if (velocity.x < 0.0)
         {
            velocity.x = -velocity.x;
         }
      }
      if (newPosition.x >= span)
      {
         newPosition.x = span - 0.0001;
         if (velocity.x > 0.0)
         {
            velocity.x = -velocity.x;
         }
      }
      if (newPosition.y <= -span)
      {
         newPosition.y = -(span - 0.0001);
         if (velocity.y < 0.0)
         {
            velocity.y = -velocity.y;
         }
      }
      if (newPosition.y >= span)
      {
         newPosition.y = span - 0.0001;
         if (velocity.y > 0.0)
         {
            velocity.y = -velocity.y;
         }
      }
      if (newPosition.z <= -span)
      {
         newPosition.z = -(span - 0.0001);
         if (velocity.z < 0.0)
         {
            velocity.z = -velocity.z;
         }
      }
      if (newPosition.z >= span)
      {
         newPosition.z = span - 0.0001;
         if (velocity.z > 0.0)
         {
            velocity.z = -velocity.z;
         }
      }

      // Step 2: Calculate new roll, pitch, and yaw of the boid
      //         which correspond to the changes in position and velocity.
      //         Remember: the boid isn't necessarily oriented in the
      //         direction of the velocity!
      // (assuming +y is up, +z is through the nose, +x is through the left wing)
      calculateRollPitchYaw(acceleration, oldVelocity, position);
   }

   // remember current velocity
   oldVelocity = velocity;

   // remember desired acceleration (the acceleration vector that the
   // Navigator() module specified
   acceleration = navigator(boidList);
}


// Move to new position.
void
Boid::move()
{
   position = newPosition;
}


// Boid constructor.
Boid::Boid(Vector bPosition, Vector bVelocity, Vector bDimensions)
{
   flightflag       = false;                      // haven't flown yet
   position         = bPosition;
   velocity         = bVelocity;
   dimensions       = bDimensions;                // width, height, length
   mass             = 9.07;                       // 9.07 kg is approx 20 lbs.
   maxVelocity      = MAX_VELOCITY;
   maxAcceleration  = MAX_ACCELERATION;
   cruiseDistance   = CRUISE_DISTANCE;
   roll             = pitch = yaw = 0;
   boidType         = NORMAL;
   flockSelectively = false;
   bodyLength       = bDimensions.z;
   boidCount++;
   boidNumber = boidCount;
   //cerr << "\nConstructing boid " << boidNumber << endl;
   sprintf(bName, "boid%d", boidCount);
   if (Magnitude(velocity) > maxVelocity)
   {
      velocity.SetMagnitude(maxVelocity);
      oldVelocity = velocity;
   }
}


// Construct specified boid.
Boid::Boid(Vector bPosition, Vector bVelocity, Vector bDimensions,
           int bBoidType, int bBoidNumber)
{
   flightflag       = false;                      // haven't flown yet
   position         = bPosition;
   velocity         = bVelocity;
   dimensions       = bDimensions;                // width, height, length
   mass             = 9.07;                       // 9.07 kg is approx 20 lbs.
   maxVelocity      = MAX_VELOCITY;
   maxAcceleration  = MAX_ACCELERATION;
   cruiseDistance   = CRUISE_DISTANCE;
   roll             = pitch = yaw = 0;
   boidType         = bBoidType;
   flockSelectively = false;
   bodyLength       = bDimensions.z;
   boidNumber       = bBoidNumber;
   //cerr << "\nConstructing boid " << boidNumber << endl;
   sprintf(bName, "boid%d", boidNumber);
   if (Magnitude(velocity) > maxVelocity)
   {
      velocity.SetMagnitude(maxVelocity);
      oldVelocity = velocity;
   }
}


// Clone boid.
Boid *Boid::clone()
{
   Boid *boid = new Boid(position, velocity, dimensions);

#ifdef _DEBUG
   assert(boid != NULL);
#endif
   boid->flightflag       = flightflag;
   boid->newPosition      = newPosition;
   boid->oldVelocity      = oldVelocity;
   boid->acceleration     = acceleration;
   boid->mass             = mass;
   boid->maxVelocity      = maxVelocity;
   boid->maxAcceleration  = maxAcceleration;
   boid->roll             = roll;
   boid->pitch            = pitch;
   boid->yaw              = yaw;
   boid->cruiseDistance   = cruiseDistance;
   boid->boidType         = boidType;
   boid->flockSelectively = flockSelectively;
   boid->bodyLength       = bodyLength;
   boid->boidNumber       = boidNumber;
   strcpy(boid->bName, bName);
   return(boid);
}


#endif                                            /* __BOID_C */
