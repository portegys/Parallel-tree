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

/*
 * Programmer : Tom Portegys <portegys@ilstu.edu>
 * File Name : processorSet.hpp
 *
 * Description : Flocking boids (alife creatures) tracked by octrees.
 *               The octrees are distributed among processors.
 *
 * Date : 3/24/2003
 */

#include "processorSet.hpp"
#include <stdlib.h>
#include <assert.h>

#define PRAND    ((float)(rand() % 1001) / 1000.0f)

// Maximum boundary movement rate for load-balancing.
const float ProcessorSet::MAX_BOUNDARY_VELOCITY = 0.1f;

// Constructor.
ProcessorSet::ProcessorSet(int dimension, float span, int numBoids,
                           int *ptids, int tid, int randomSeed)
{
   register int   i, j, k, proc;
   float          size;
   Octree::BOUNDS bounds;
   int            *assign;

   // Octree dimension must be power of 2.
   for (i = 1; i < dimension; i = 2 * i)
   {
   }
   assert(i == dimension);
   this->dimension = dimension;
   this->span      = span;
   Boid::setSpan(span);
   margin = span / 20.0f;
   Boid::setMargin(margin);
   this->numBoids = numBoids;
   numProcs       = dimension * dimension * dimension;
   this->ptids    = new int[numProcs];
#ifdef _DEBUG
   assert(this->ptids != NULL);
#endif
   for (i = 0; i < numProcs; i++)
   {
      this->ptids[i] = ptids[i];
   }
   this->tid = tid;
   msgSent   = msgRcv = 0;

   // Randomly assign boids to local octrees.
   srand(randomSeed);
   assign = new int[numProcs];
   for (i = 0; i < numProcs; i++)
   {
      assign[i] = 0;
   }
   for (i = 0; i < numBoids; )
   {
      j = rand() % numProcs;
      if (this->ptids[j] == tid)
      {
         assign[j]++;
         i++;
      }
   }

   // Create octrees.
   octrees = new Octree *[numProcs];
#ifdef _DEBUG
   assert(octrees != NULL);
#endif
   migrations = new OctObject *[numProcs];
#ifdef _DEBUG
   assert(migrations != NULL);
#endif
   size = (span * 2.0f) / (float)dimension;
   for (i = 0; i < dimension; i++)
   {
      for (j = 0; j < dimension; j++)
      {
         for (k = 0; k < dimension; k++)
         {
            proc          = (i * dimension * dimension) + (j * dimension) + k;
            octrees[proc] = new Octree(0.0f, 0.0f, 0.0f, span, PRECISION);
#ifdef _DEBUG
            assert(octrees[proc] != NULL);
#endif

            // Set octree bounds.
            bounds.xmin = ((float)k * size) - span;
            bounds.xmax = bounds.xmin + size;
            bounds.ymin = ((float)j * size) - span;
            bounds.ymax = bounds.ymin + size;
            bounds.zmin = ((float)i * size) - span;
            bounds.zmax = bounds.zmin + size;
            octrees[proc]->setBounds(bounds);

            // Create boids.
            if (this->ptids[proc] == tid)
            {
               makeBoids(proc, assign[proc]);
            }
         }
      }
   }
   delete assign;

   newBounds = new Octree::BOUNDS[numProcs];
#ifdef _DEBUG
   assert(newBounds != NULL);
#endif

   // Load-balancing off.
   loadBalance = false;
}


// Destructor.
ProcessorSet::~ProcessorSet()
{
   register int i;

   for (i = 0; i < numProcs; i++)
   {
      delete octrees[i];
   }
   delete octrees;
   delete migrations;
   delete ptids;
   delete newBounds;
}


// Make boids for processor.
void ProcessorSet::makeBoids(int proc, int quantity)
{
   Boid      *boid;
   OctObject *object;
   Vector    position;
   Vector    velocity;
   Vector    dimensions(1, .2, .75);              // dimensions of boid (RAD, height, length)
   double    diameter;

   for (int i = 0; i < quantity; i++)
   {
      // Set up position and velocity.
      velocity = Vector(rand() % int(span), rand() % int(span), rand() % int(span));
      velocity.SetMagnitude(PRAND);
      diameter   = octrees[proc]->bounds.xmax - octrees[proc]->bounds.xmin;
      position.x = (PRAND * diameter) + octrees[proc]->bounds.xmin;
      diameter   = octrees[proc]->bounds.ymax - octrees[proc]->bounds.ymin;
      position.y = (PRAND * diameter) + octrees[proc]->bounds.ymin;
      diameter   = octrees[proc]->bounds.zmax - octrees[proc]->bounds.zmin;
      position.z = (PRAND * diameter) + octrees[proc]->bounds.zmin;
      boid       = new Boid(position, velocity, dimensions);
#ifdef _DEBUG
      assert(boid != NULL);
#endif
      // Insert into octree.
      object = new OctObject((float)(position.x), (float)(position.y),
                             (float)(position.z), (void *)boid);
#ifdef _DEBUG
      assert(object != NULL);
#endif
      octrees[proc]->insert(object);
   }
}


// Run.
void ProcessorSet::run()
{
   int operation, proc;

   // Message loop.
#ifdef UNIX
   while (true)
   {
      pvm_recv(-1, 0);
      pvm_upkint(&operation, 1, 1);
      switch (operation)
      {
      case AIM:
         aim();
         break;

      case MOVE:
         move();
         break;

      case REPORT:
         report();
         break;

      case BALANCE:
         // Set load-balanced bounds.
         for (proc = 0; proc < numProcs; proc++)
         {
            pvm_upkfloat(&(octrees[proc]->bounds.xmin), 1, 1);
            pvm_upkfloat(&(octrees[proc]->bounds.xmax), 1, 1);
            pvm_upkfloat(&(octrees[proc]->bounds.ymin), 1, 1);
            pvm_upkfloat(&(octrees[proc]->bounds.ymax), 1, 1);
            pvm_upkfloat(&(octrees[proc]->bounds.zmin), 1, 1);
            pvm_upkfloat(&(octrees[proc]->bounds.zmax), 1, 1);
         }
         ready();
         break;

      case MIGRATE:
         // Migrate boids.
         migrate();
         ready();
         break;

      case STATS:
         stats();
         break;

      case QUIT:
         pvm_exit();
         exit(0);

      default:
         serveClient(operation);
      }
   }
#endif
}


// Aim: update velocity and acceleration and determine new position.
void ProcessorSet::aim()
{
   register int       i, proc;
   register OctObject *object;
   register Boid      *boidList, *boid, *boid2;
   Octree::BOUNDS     bounds;

   for (proc = 0; proc < numProcs; proc++)
   {
      // Update local objects.
      if (ptids[proc] != tid)
      {
         continue;
      }

      for (object = octrees[proc]->objects; object != NULL; object = object->next)
      {
         // Do cross-processor search.
         boidList    = NULL;
         bounds.xmin = object->position.m_x - Boid::visibilityRange;
         bounds.xmax = object->position.m_x + Boid::visibilityRange;
         bounds.ymin = object->position.m_y - Boid::visibilityRange;
         bounds.ymax = object->position.m_y + Boid::visibilityRange;
         bounds.zmin = object->position.m_z - Boid::visibilityRange;
         bounds.zmax = object->position.m_z + Boid::visibilityRange;
         for (i = 0; i < numProcs; i++)
         {
            // Search intersects processor space?
            if (intersects(octrees[i]->bounds, bounds))
            {
               // Accumulate search results.
               boid = search(i, object->position, Boid::visibilityRange);
               while (boid != NULL)
               {
                  boid2      = boid->next;
                  boid->next = boidList;
                  boidList   = boid;
                  boid       = boid2;
               }
            }
         }

         // Update boid based on search results.
         boid = (Boid *)object->client;
         boid->aim(boidList);

         // Free search elements.
         while (boidList != NULL)
         {
            boid     = boidList;
            boidList = boidList->next;
            delete boid;
         }
      }
   }

   // Report update done to master.
   ready();
}


// Move boids.
void ProcessorSet::move()
{
   register int       i, proc;
   register OctObject *object, *object2;
   register Boid      *boid;
   Vector             position;

   for (proc = 0; proc < numProcs; proc++)
   {
      // Update local objects.
      if (ptids[proc] != tid)
      {
         continue;
      }

      for (object = octrees[proc]->objects, object2 = NULL; object != NULL; )
      {
         boid = (Boid *)object->client;
         boid->move();
         position = boid->getPosition();
         if (!object->move((float)position.x, (float)position.y, (float)position.z))
         {
            // Boid migrating processors.
            octrees[proc]->load--;
            if (object2 == NULL)
            {
               octrees[proc]->objects = object->next;
            }
            else
            {
               object2->next = object->next;
            }
            object->next = NULL;
            for (i = 0; i < numProcs; i++)
            {
               if (i == proc)
               {
                  continue;
               }
               if (object->isInside(octrees[i]))
               {
                  insert(i, boid);
                  break;
               }
            }
#ifdef _DEBUG
            assert(i < numProcs);
#endif
            delete object;
            if (object2 == NULL)
            {
               object = octrees[proc]->objects;
            }
            else
            {
               object = object2->next;
            }
         }
         else
         {
            object2 = object;
            object  = object->next;
         }
      }
   }

   // Report update done to master.
   ready();
}


// Report load.
void ProcessorSet::report()
{
   int operation, proc;

   operation = REPORT_RESULT;
   for (proc = 0; proc < numProcs; proc++)
   {
#ifdef UNIX
      if (ptids[proc] != tid)
      {
         continue;
      }
      pvm_initsend(PvmDataDefault);
      pvm_pkint(&operation, 1, 1);
      pvm_pkint(&proc, 1, 1);
      pvm_pkint(&(octrees[proc]->load), 1, 1);
      octrees[proc]->findMedian();
      pvm_pkfloat(&(octrees[proc]->median.m_x), 1, 1);
      pvm_pkfloat(&(octrees[proc]->median.m_y), 1, 1);
      pvm_pkfloat(&(octrees[proc]->median.m_z), 1, 1);
      pvm_send(pvm_parent(), 0);
#endif
   }
}


// Report statistics.
void ProcessorSet::stats()
{
   int operation, proc, load;

   operation = STATS_RESULT;
#ifdef UNIX
   pvm_initsend(PvmDataDefault);
   pvm_pkint(&operation, 1, 1);
   pvm_pkint(&msgSent, 1, 1);
   pvm_pkint(&msgRcv, 1, 1);
   for (proc = load = 0; proc < numProcs; proc++)
   {
      if (ptids[proc] != tid)
      {
         continue;
      }
      load += octrees[proc]->load;
   }
   pvm_pkint(&load, 1, 1);
   pvm_send(pvm_parent(), 0);
#endif
   msgSent = msgRcv = 0;
}


// Report readiness to master.
void ProcessorSet::ready()
{
#ifdef UNIX
   int operation, proc;

   operation = READY;
   for (proc = 0; proc < numProcs; proc++)
   {
      if (ptids[proc] != tid)
      {
         continue;
      }
      pvm_initsend(PvmDataDefault);
      pvm_pkint(&operation, 1, 1);
      pvm_pkint(&proc, 1, 1);
      pvm_send(pvm_parent(), 0);
   }
#endif
}


// Insert boid into a processor.
bool ProcessorSet::insert(int proc, Boid *boid)
{
   if (ptids[proc] == tid)
   {
      // Local insert.
      Vector    position = boid->getPosition();
      OctObject *object  = new OctObject((float)(position.x),
                                         (float)(position.y), (float)(position.z), (void *)boid);
      return(octrees[proc]->insert(object));
   }
   else
   {
      // Send remote insert.
      int    i;
      Vector v;
      float  x, y, z;
#ifdef UNIX
      int operation, result, updated;
      pvm_initsend(PvmDataDefault);
      operation = INSERT;
      pvm_pkint(&operation, 1, 1);
      pvm_pkint(&proc, 1, 1);
      v = boid->getPosition();
      x = (float)(v.x);
      y = (float)(v.y);
      z = (float)(v.z);
      pvm_pkfloat(&x, 1, 1);
      pvm_pkfloat(&y, 1, 1);
      pvm_pkfloat(&z, 1, 1);
      v = boid->getVelocity();
      x = (float)(v.x);
      y = (float)(v.y);
      z = (float)(v.z);
      pvm_pkfloat(&x, 1, 1);
      pvm_pkfloat(&y, 1, 1);
      pvm_pkfloat(&z, 1, 1);
      v = boid->getDimensions();
      x = (float)(v.x);
      y = (float)(v.y);
      z = (float)(v.z);
      pvm_pkfloat(&x, 1, 1);
      pvm_pkfloat(&y, 1, 1);
      pvm_pkfloat(&z, 1, 1);
      i = boid->getBoidType();
      pvm_pkint(&i, 1, 1);
      i = boid->getBoidNumber();
      pvm_pkint(&i, 1, 1);
      pvm_send(ptids[proc], 0);
      msgSent++;
#endif
      // Delete boid.
      delete boid;

      // For speed, assume successful.
      return(true);
   }
}


// Search a processor.
// Returns list of matching boids.
Boid *ProcessorSet::search(int proc, Point3D point, float radius)
{
   register OctObject *object;
   register Boid      *boidList, *boid, *proxyBoid;

   // Local search?
   if (ptids[proc] == tid)
   {
      // Local search.
      boidList = NULL;
      object   = octrees[proc]->search(point, radius);
      while (object != NULL)
      {
         boid      = (Boid *)object->client;
         proxyBoid = boid->clone();
#ifdef _DEBUG
         assert(proxyBoid != NULL);
#endif
         proxyBoid->next = boidList;
         boidList        = proxyBoid;
         object          = object->retnext;
      }
      return(boidList);
   }
   else
   {
      // Remote search.
      boidList = NULL;
#ifdef UNIX
      register int i;
      int          operation, result, packets, size;
      bool         firstpacket;
      Vector       pos, vel, dim;
      float        x, y, z;
      int          type, num;

      pvm_initsend(PvmDataDefault);
      operation = SEARCH;
      pvm_pkint(&operation, 1, 1);
      pvm_pkint(&tid, 1, 1);
      pvm_pkint(&proc, 1, 1);
      pvm_pkfloat(&point.m_x, 1, 1);
      pvm_pkfloat(&point.m_y, 1, 1);
      pvm_pkfloat(&point.m_z, 1, 1);
      pvm_pkfloat(&radius, 1, 1);
      pvm_send(ptids[proc], 0);
      msgSent++;

      // Get search results.
      firstpacket = false;
      while (!firstpacket || packets > 0)
      {
         pvm_recv(-1, 0);
         msgRcv++;
         pvm_upkint(&operation, 1, 1);
         switch (operation)
         {
         case SEARCH_RESULT:
            break;

         case QUIT:
            pvm_exit();
            exit(0);

         default:
            // Serve client request.
            serveClient(operation);
            continue;
         }

         // Unpack search result packet.
         if (!firstpacket)
         {
            pvm_upkint(&packets, 1, 1);
            firstpacket = true;
         }
         pvm_upkint(&size, 1, 1);
         for (i = 0; i < size; i++)
         {
            pvm_upkfloat(&x, 1, 1);
            pvm_upkfloat(&y, 1, 1);
            pvm_upkfloat(&z, 1, 1);
            pos.x = (double)x;
            pos.y = (double)y;
            pos.z = (double)z;
            pvm_upkfloat(&x, 1, 1);
            pvm_upkfloat(&y, 1, 1);
            pvm_upkfloat(&z, 1, 1);
            vel.x = (double)x;
            vel.y = (double)y;
            vel.z = (double)z;
            pvm_upkfloat(&x, 1, 1);
            pvm_upkfloat(&y, 1, 1);
            pvm_upkfloat(&z, 1, 1);
            dim.x = (double)x;
            dim.y = (double)y;
            dim.z = (double)z;
            pvm_upkint(&type, 1, 1);
            pvm_upkint(&num, 1, 1);
            proxyBoid = new Boid(pos, vel, dim, type, num);
#ifdef _DEBUG
            assert(proxyBoid != NULL);
#endif
            proxyBoid->next = boidList;
            boidList        = proxyBoid;
         }
         packets--;
      }
#endif
      return(boidList);
   }
}


// Serve client processors.
void ProcessorSet::serveClient(int operation)
{
#ifdef UNIX
   int                   i, j, proc, retOp, packets, size, updated, rtid;
   Vector                pos, vel, dim;
   int                   type, num;
   Point3D               position;
   float                 radius, x, y, z;
   register Boid         *boid, *boidList;
   register OctObject    *object;
   struct Frustum::Plane planes[6];
   struct Frustum        *frustum;
   register VISIBLE      *visibleList, *visibleElem;

   switch (operation)
   {
   // Insert.
   case INSERT:
      pvm_upkint(&proc, 1, 1);
#ifdef _DEBUG
      assert(ptids[proc] == tid);
#endif
      pvm_upkfloat(&x, 1, 1);
      pvm_upkfloat(&y, 1, 1);
      pvm_upkfloat(&z, 1, 1);
      pos.x = (double)x;
      pos.y = (double)y;
      pos.z = (double)z;
      pvm_upkfloat(&x, 1, 1);
      pvm_upkfloat(&y, 1, 1);
      pvm_upkfloat(&z, 1, 1);
      vel.x = (double)x;
      vel.y = (double)y;
      vel.z = (double)z;
      pvm_upkfloat(&x, 1, 1);
      pvm_upkfloat(&y, 1, 1);
      pvm_upkfloat(&z, 1, 1);
      dim.x = (double)x;
      dim.y = (double)y;
      dim.z = (double)z;
      pvm_upkint(&type, 1, 1);
      pvm_upkint(&num, 1, 1);
      boid = new Boid(pos, vel, dim, type, num);
#ifdef _DEBUG
      assert(boid != NULL);
#endif
      object = new OctObject((float)(pos.x), (float)(pos.y), (float)(pos.z), (void *)boid);
#ifdef _DEBUG
      assert(object != NULL);
#endif
      octrees[proc]->insert(object);
      break;

   // Search.
   case SEARCH:
      pvm_upkint(&rtid, 1, 1);
      pvm_upkint(&proc, 1, 1);
#ifdef _DEBUG
      assert(ptids[proc] == tid);
#endif
      pvm_upkfloat(&position.m_x, 1, 1);
      pvm_upkfloat(&position.m_y, 1, 1);
      pvm_upkfloat(&position.m_z, 1, 1);
      pvm_upkfloat(&radius, 1, 1);

      // Search.
      boidList = search(proc, position, radius);

      // Send results.
      retOp = SEARCH_RESULT;
      for (boid = boidList, size = 0; boid != NULL;
           boid = boid->next, size++)
      {
      }
      pvm_initsend(PvmDataDefault);
      pvm_pkint(&retOp, 1, 1);
      if ((size % MAX_MESSAGE_ITEMS) == 0)
      {
         packets = size / MAX_MESSAGE_ITEMS;
         if (packets == 0)
         {
            packets = 1;
         }
      }
      else
      {
         packets = size / MAX_MESSAGE_ITEMS;
         packets++;
      }
      pvm_pkint(&packets, 1, 1);
      for (i = 0; i < packets; i++)
      {
         for (boid = boidList, size = 0; boid != NULL;
              boid = boid->next, size++)
         {
         }
         if (size > MAX_MESSAGE_ITEMS)
         {
            size = MAX_MESSAGE_ITEMS;
         }
         if (i > 0)
         {
            pvm_initsend(PvmDataDefault);
            pvm_pkint(&retOp, 1, 1);
         }
         pvm_pkint(&size, 1, 1);
         for (j = 0; j < size; j++)
         {
            pos  = boidList->getPosition();
            vel  = boidList->getVelocity();
            dim  = boidList->getDimensions();
            type = boidList->getBoidType();
            num  = boidList->getBoidNumber();
            x    = (float)(pos.x);
            y    = (float)(pos.y);
            z    = (float)(pos.z);
            pvm_pkfloat(&x, 1, 1);
            pvm_pkfloat(&y, 1, 1);
            pvm_pkfloat(&z, 1, 1);
            x = (float)(vel.x);
            y = (float)(vel.y);
            z = (float)(vel.z);
            pvm_pkfloat(&x, 1, 1);
            pvm_pkfloat(&y, 1, 1);
            pvm_pkfloat(&z, 1, 1);
            x = (float)(dim.x);
            y = (float)(dim.y);
            z = (float)(dim.z);
            pvm_pkfloat(&x, 1, 1);
            pvm_pkfloat(&y, 1, 1);
            pvm_pkfloat(&z, 1, 1);
            pvm_pkint(&type, 1, 1);
            pvm_pkint(&num, 1, 1);
            boid     = boidList;
            boidList = boidList->next;
            delete boid;
         }
         pvm_send(rtid, 0);
         msgSent++;
      }
      break;

   // Search for visible objects.
   case VIEW:
      pvm_upkint(&proc, 1, 1);
#ifdef _DEBUG
      assert(ptids[proc] == tid);
#endif
      for (i = 0; i < 6; i++)
      {
         pvm_upkfloat(&planes[i].a, 1, 1);
         pvm_upkfloat(&planes[i].b, 1, 1);
         pvm_upkfloat(&planes[i].c, 1, 1);
         pvm_upkfloat(&planes[i].d, 1, 1);
      }
      frustum = new Frustum(planes);
#ifdef _DEBUG
      assert(frustum != NULL);
#endif

      // Search.
      visibleList = searchVisible(frustum);

      // Send results.
      retOp = VIEW_RESULT;
      for (visibleElem = visibleList, size = 0; visibleElem != NULL;
           visibleElem = visibleElem->next, size++)
      {
      }
      pvm_initsend(PvmDataDefault);
      pvm_pkint(&retOp, 1, 1);
      if ((size % MAX_MESSAGE_ITEMS) == 0)
      {
         packets = size / MAX_MESSAGE_ITEMS;
         if (packets == 0)
         {
            packets = 1;
         }
      }
      else
      {
         packets = size / MAX_MESSAGE_ITEMS;
         packets++;
      }
      pvm_pkint(&packets, 1, 1);
      for (i = 0; i < packets; i++)
      {
         for (visibleElem = visibleList, size = 0; visibleElem != NULL;
              visibleElem = visibleElem->next, size++)
         {
         }
         if (size > MAX_MESSAGE_ITEMS)
         {
            size = MAX_MESSAGE_ITEMS;
         }
         if (i > 0)
         {
            pvm_initsend(PvmDataDefault);
            pvm_pkint(&retOp, 1, 1);
         }
         pvm_pkint(&size, 1, 1);
         for (j = 0; j < size; j++)
         {
            pvm_pkint(&(visibleList->id), 1, 1);
            pvm_pkfloat(&(visibleList->position.m_x), 1, 1);
            pvm_pkfloat(&(visibleList->position.m_y), 1, 1);
            pvm_pkfloat(&(visibleList->position.m_z), 1, 1);
            pvm_pkfloat(&(visibleList->velocity.m_x), 1, 1);
            pvm_pkfloat(&(visibleList->velocity.m_y), 1, 1);
            pvm_pkfloat(&(visibleList->velocity.m_z), 1, 1);
            visibleElem = visibleList;
            visibleList = visibleList->next;
            delete visibleElem;
         }
         pvm_send(pvm_parent(), 0);
      }
      break;

   default:
#ifdef _DEBUG
      assert(false);
#endif
   }
#endif
}


// Search for visible local objects.
ProcessorSet::VISIBLE *ProcessorSet::searchVisible(Frustum *frustum)
{
   register int       proc;
   register OctObject *object;
   register VISIBLE   *visible, *visibleList;
   Vector             velocity;

   visibleList = NULL;
   for (proc = 0; proc < numProcs; proc++)
   {
      if (ptids[proc] == tid)
      {
         object = octrees[proc]->searchVisible(frustum);
         while (object != NULL)
         {
            visible = new VISIBLE;
#ifdef _DEBUG
            assert(visible != NULL);
#endif
            visible->id           = ((Boid *)(object->client))->getBoidNumber();
            visible->position     = object->position;
            velocity              = ((Boid *)(object->client))->getVelocity();
            visible->velocity.m_x = velocity.x;
            visible->velocity.m_y = velocity.y;
            visible->velocity.m_z = velocity.z;
            visible->next         = visibleList;
            visibleList           = visible;
            object = object->retnext;
         }
      }
   }
   return(visibleList);
}


// Load-balance.
void ProcessorSet::balance()
{
   register int      proc;
   Octree::BOUNDS    bounds;
   register CENTROID *centroids, *centroid;
   int               *parray;

   // Load-balance.
   bounds.xmax = bounds.ymax = bounds.zmax = span;
   bounds.xmin = bounds.ymin = bounds.zmin = -span;
   centroids   = NULL;
   for (proc = 0; proc < numProcs; proc++)
   {
      centroid = new CENTROID;
#ifdef _DEBUG
      assert(centroid != NULL);
#endif
      centroid->next     = centroids;
      centroids          = centroid;
      centroid->load     = octrees[proc]->load;
      centroid->position = octrees[proc]->median;
   }
   parray = new int[numProcs];
#ifdef _DEBUG
   assert(parray != NULL);
#endif
   for (proc = 0; proc < numProcs; proc++)
   {
      parray[proc] = proc;
   }
   balance(parray, dimension, dimension, dimension, XCUT, bounds, centroids);
   while (centroids != NULL)
   {
      centroid  = centroids;
      centroids = centroids->next;
      delete centroid;
   }
   delete parray;
   for (proc = 0; proc < numProcs; proc++)
   {
      octrees[proc]->setBounds(newBounds[proc]);
   }

   // Migrate boids to their proper processors.
   migrate();
}


// Load-balance subroutine.
void ProcessorSet::balance(int *parray, int rows, int columns, int ranks,
                           CUT cut, Octree::BOUNDS bounds, CENTROID *centroids)
{
   register int      i, j, k, proc;
   register CENTROID *centroid, *centroid2, *subCentroids, *subCentroid;
   Octree::BOUNDS    subBounds;
   int               *subParray;
   float             d, mid, c, c2;

   // Save bounds?
   if ((rows == 1) && (columns == 1) && (ranks == 1))
   {
      newBounds[parray[0]] = bounds;
      return;
   }

   // Bisect on median.
   sortCentroids(&centroids, cut);
   d = 0.0;
   for (centroid = centroids, i = 0; centroid != NULL; centroid = centroid->next)
   {
      i += centroid->load;
   }
   mid = (float)i / 2.0f;
   for (centroid = centroids, centroid2 = NULL; centroid != NULL;
        centroid2 = centroid, centroid = centroid->next)
   {
      d += centroid->load;
      if (d >= mid)
      {
         break;
      }
   }
   switch (cut)
   {
   case XCUT:
      if (centroid2 == NULL)
      {
         c2 = bounds.xmin;
      }
      else
      {
         c2 = centroid2->position.m_x;
      }
      if (centroid == NULL)
      {
         c = bounds.xmax / 2.0f;
      }
      else
      {
         c = centroid->position.m_x;
      }
      break;

   case YCUT:
      if (centroid2 == NULL)
      {
         c2 = bounds.ymin;
      }
      else
      {
         c2 = centroid2->position.m_y;
      }
      if (centroid == NULL)
      {
         c = bounds.ymax / 2.0f;
      }
      else
      {
         c = centroid->position.m_y;
      }
      break;

   case ZCUT:
      if (centroid2 == NULL)
      {
         c2 = bounds.zmin;
      }
      else
      {
         c2 = centroid2->position.m_z;
      }
      if (centroid == NULL)
      {
         c = bounds.zmax / 2.0f;
      }
      else
      {
         c = centroid->position.m_z;
      }
      break;
   }
   if (d > mid)
   {
      d   = (mid - (d - centroid->load)) / centroid->load;
      mid = c2 + (d * (c - c2));
   }
   else
   {
      mid = c;
   }

   // Lesser bisection.
   subBounds = bounds;
   switch (cut)
   {
   case XCUT:
      d = octrees[parray[columns / 2]]->bounds.xmin;
      if (mid > d)
      {
         if ((mid - d) > MAX_BOUNDARY_VELOCITY)
         {
            mid = d + MAX_BOUNDARY_VELOCITY;
         }
      }
      else
      {
         if ((d - mid) > MAX_BOUNDARY_VELOCITY)
         {
            mid = d - MAX_BOUNDARY_VELOCITY;
         }
      }
      subBounds.xmax = mid;
      if (subBounds.xmax > bounds.xmax)
      {
         subBounds.xmax = bounds.xmax;
      }
      if (subBounds.xmax < bounds.xmin)
      {
         subBounds.xmax = bounds.xmin;
      }
      break;

   case YCUT:
      d = octrees[parray[(rows / 2) * columns]]->bounds.ymin;
      if (mid > d)
      {
         if ((mid - d) > MAX_BOUNDARY_VELOCITY)
         {
            mid = d + MAX_BOUNDARY_VELOCITY;
         }
      }
      else
      {
         if ((d - mid) > MAX_BOUNDARY_VELOCITY)
         {
            mid = d - MAX_BOUNDARY_VELOCITY;
         }
      }
      subBounds.ymax = mid;
      if (subBounds.ymax > bounds.ymax)
      {
         subBounds.ymax = bounds.ymax;
      }
      if (subBounds.ymax < bounds.ymin)
      {
         subBounds.ymax = bounds.ymin;
      }
      break;

   case ZCUT:
      d = octrees[parray[(ranks / 2) * rows * columns]]->bounds.zmin;
      if (mid > d)
      {
         if ((mid - d) > MAX_BOUNDARY_VELOCITY)
         {
            mid = d + MAX_BOUNDARY_VELOCITY;
         }
      }
      else
      {
         if ((d - mid) > MAX_BOUNDARY_VELOCITY)
         {
            mid = d - MAX_BOUNDARY_VELOCITY;
         }
      }
      subBounds.zmax = mid;
      if (subBounds.zmax > bounds.zmax)
      {
         subBounds.zmax = bounds.zmax;
      }
      if (subBounds.zmax < bounds.zmin)
      {
         subBounds.zmax = bounds.zmin;
      }
      break;
   }
   subParray = new int[ranks * rows * columns / 2];
#ifdef _DEBUG
   assert(subParray != NULL);
#endif
   subCentroids = NULL;
   switch (cut)
   {
   case XCUT:
      for (centroid = centroids; centroid != NULL; centroid = centroid->next)
      {
         if (centroid->position.m_x < subBounds.xmax)
         {
            subCentroid = new CENTROID;
#ifdef _DEBUG
            assert(subCentroid != NULL);
#endif
            subCentroid->next     = subCentroids;
            subCentroids          = subCentroid;
            subCentroid->position = centroid->position;
            subCentroid->load     = centroid->load;
         }
      }
      for (i = proc = 0; i < ranks; i++)
      {
         for (j = 0; j < rows; j++)
         {
            for (k = 0; k < columns / 2; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      balance(subParray, rows, columns / 2, ranks, YCUT, subBounds, subCentroids);
      break;

   case YCUT:
      for (centroid = centroids; centroid != NULL; centroid = centroid->next)
      {
         if (centroid->position.m_y < subBounds.ymax)
         {
            subCentroid = new CENTROID;
#ifdef _DEBUG
            assert(subCentroid != NULL);
#endif
            subCentroid->next     = subCentroids;
            subCentroids          = subCentroid;
            subCentroid->position = centroid->position;
            subCentroid->load     = centroid->load;
         }
      }
      for (i = proc = 0; i < ranks; i++)
      {
         for (j = 0; j < rows / 2; j++)
         {
            for (k = 0; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      balance(subParray, rows / 2, columns, ranks, ZCUT, subBounds, subCentroids);
      break;

   case ZCUT:
      for (centroid = centroids; centroid != NULL; centroid = centroid->next)
      {
         if (centroid->position.m_z < subBounds.zmax)
         {
            subCentroid = new CENTROID;
#ifdef _DEBUG
            assert(subCentroid != NULL);
#endif
            subCentroid->next     = subCentroids;
            subCentroids          = subCentroid;
            subCentroid->position = centroid->position;
            subCentroid->load     = centroid->load;
         }
      }
      for (i = proc = 0; i < ranks / 2; i++)
      {
         for (j = 0; j < rows; j++)
         {
            for (k = 0; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      balance(subParray, rows, columns, ranks / 2, XCUT, subBounds, subCentroids);
      break;
   }
   while (subCentroids != NULL)
   {
      centroid     = subCentroids;
      subCentroids = subCentroids->next;
      delete centroid;
   }

   // Greater bisection.
   switch (cut)
   {
   case XCUT:
      subBounds.xmin = subBounds.xmax;
      subBounds.xmax = bounds.xmax;
      break;

   case YCUT:
      subBounds.ymin = subBounds.ymax;
      subBounds.ymax = bounds.ymax;
      break;

   case ZCUT:
      subBounds.zmin = subBounds.zmax;
      subBounds.zmax = bounds.zmax;
      break;
   }
   subCentroids = NULL;
   switch (cut)
   {
   case XCUT:
      for (centroid = centroids; centroid != NULL; centroid = centroid->next)
      {
         if (centroid->position.m_x >= subBounds.xmin)
         {
            subCentroid = new CENTROID;
#ifdef _DEBUG
            assert(subCentroid != NULL);
#endif
            subCentroid->next     = subCentroids;
            subCentroids          = subCentroid;
            subCentroid->position = centroid->position;
            subCentroid->load     = centroid->load;
         }
      }
      for (i = proc = 0; i < ranks; i++)
      {
         for (j = 0; j < rows; j++)
         {
            for (k = columns / 2; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      balance(subParray, rows, columns / 2, ranks, YCUT, subBounds, subCentroids);
      break;

   case YCUT:
      for (centroid = centroids; centroid != NULL; centroid = centroid->next)
      {
         if (centroid->position.m_y >= subBounds.ymin)
         {
            subCentroid = new CENTROID;
#ifdef _DEBUG
            assert(subCentroid != NULL);
#endif
            subCentroid->next     = subCentroids;
            subCentroids          = subCentroid;
            subCentroid->position = centroid->position;
            subCentroid->load     = centroid->load;
         }
      }
      for (i = 0, proc = 0; i < ranks; i++)
      {
         for (j = rows / 2; j < rows; j++)
         {
            for (k = 0; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      balance(subParray, rows / 2, columns, ranks, ZCUT, subBounds, subCentroids);
      break;

   case ZCUT:
      for (centroid = centroids; centroid != NULL; centroid = centroid->next)
      {
         if (centroid->position.m_z >= subBounds.zmin)
         {
            subCentroid = new CENTROID;
#ifdef _DEBUG
            assert(subCentroid != NULL);
#endif
            subCentroid->next     = subCentroids;
            subCentroids          = subCentroid;
            subCentroid->position = centroid->position;
            subCentroid->load     = centroid->load;
         }
      }
      for (i = ranks / 2, proc = 0; i < ranks; i++)
      {
         for (j = 0; j < rows; j++)
         {
            for (k = 0; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      balance(subParray, rows, columns, ranks / 2, XCUT, subBounds, subCentroids);
      break;
   }
   delete subParray;
   while (subCentroids != NULL)
   {
      centroid     = subCentroids;
      subCentroids = subCentroids->next;
      delete centroid;
   }
}


// Sort centroids.
void ProcessorSet::sortCentroids(CENTROID **centroids, CUT cut)
{
   register CENTROID *head, *c, *c2, *c3;

   head = NULL;
   while (*centroids != NULL)
   {
      for (c = head, c2 = NULL; c != NULL; c2 = c, c = c->next)
      {
         if ((cut == XCUT) && (c->position.m_x >= (*centroids)->position.m_x))
         {
            break;
         }
         if ((cut == YCUT) && (c->position.m_y >= (*centroids)->position.m_y))
         {
            break;
         }
         if ((cut == ZCUT) && (c->position.m_z >= (*centroids)->position.m_z))
         {
            break;
         }
      }
      c3         = *centroids;
      *centroids = (*centroids)->next;
      c3->next   = c;
      if (c2 == NULL)
      {
         head = c3;
      }
      else
      {
         c2->next = c3;
      }
   }
   *centroids = head;
}


// Bounding boxes intersect?
bool ProcessorSet::intersects(Octree::BOUNDS b1, Octree::BOUNDS b2)
{
   if (b1.xmax < b2.xmin)
   {
      return(false);
   }
   if (b2.xmin > b2.xmax)
   {
      return(false);
   }
   if (b1.ymax < b2.ymin)
   {
      return(false);
   }
   if (b2.ymin > b2.ymax)
   {
      return(false);
   }
   if (b1.zmax < b2.zmin)
   {
      return(false);
   }
   if (b2.zmin > b2.zmax)
   {
      return(false);
   }
   return(true);
}


// Partition processors among machines such that adjacent processors are clustered.
void ProcessorSet::partition(int *assign, int numMachines, int dimension)
{
   register int   i, j, k, proc, numprocs;
   float          span, size;
   int            *marray, *parray;
   Octree::BOUNDS bounds, *procBounds;

   // Dimension must be power of 2.
   for (i = 1; i < dimension; i = 2 * i)
   {
   }
   assert(i == dimension);

   // Must have one or even number of machines.
   assert(numMachines == 1 || (numMachines % 2) == 0);

   marray = new int[numMachines];
#ifdef _DEBUG
   assert(marray != NULL);
#endif
   for (i = 0; i < numMachines; i++)
   {
      marray[i] = i;
   }
   numprocs = dimension * dimension * dimension;
   parray   = new int[numprocs];
#ifdef _DEBUG
   assert(parray != NULL);
#endif
   for (i = 0; i < numprocs; i++)
   {
      parray[i] = i;
   }
   span        = 100.0f;
   bounds.xmax = bounds.ymax = bounds.zmax = span;
   bounds.xmin = bounds.ymin = bounds.zmin = -span;
   procBounds  = new Octree::BOUNDS[numprocs];
#ifdef _DEBUG
   assert(procBounds != NULL);
#endif
   size = (span * 2.0f) / (float)dimension;
   for (i = 0; i < dimension; i++)
   {
      for (j = 0; j < dimension; j++)
      {
         for (k = 0; k < dimension; k++)
         {
            proc = (i * dimension * dimension) + (j * dimension) + k;
            procBounds[proc].xmin = ((float)k * size) - span;
            procBounds[proc].xmax = procBounds[proc].xmin + size;
            procBounds[proc].ymin = ((float)j * size) - span;
            procBounds[proc].ymax = procBounds[proc].ymin + size;
            procBounds[proc].zmin = ((float)i * size) - span;
            procBounds[proc].zmax = procBounds[proc].zmin + size;
         }
      }
   }
   subPartition(assign, marray, numMachines, parray,
                dimension, dimension, dimension, XCUT, bounds, procBounds);
   delete marray;
   delete parray;
   delete procBounds;
}


// Partition processors among machines - subroutine.
void ProcessorSet::subPartition(int *assign, int *marray, int msize, int *parray,
                                int rows, int columns, int ranks, CUT cut,
                                Octree::BOUNDS bounds, Octree::BOUNDS *procBounds)
{
   register int   i, j, k, proc, subMsize;
   Octree::BOUNDS subBounds;
   int            *subMarray;
   int            *subParray;

   // Assign machine to processor?
   if ((rows == 1) && (columns == 1) && (ranks == 1))
   {
      assign[parray[0]] = marray[0];
      return;
   }

   // Lesser bisection.
   subBounds = bounds;
   if (msize == 1)
   {
      subMsize = 1;
   }
   else
   {
      subMsize = msize / 2;
   }
   subMarray = new int[subMsize];
#ifdef _DEBUG
   assert(subMarray != NULL);
#endif
   for (i = 0; i < subMsize; i++)
   {
      subMarray[i] = marray[i];
   }
   subParray = new int[ranks * rows * columns / 2];
#ifdef _DEBUG
   assert(subParray != NULL);
#endif
   switch (cut)
   {
   case XCUT:
      subBounds.xmax = procBounds[parray[columns / 2]].xmin;
      for (i = proc = 0; i < ranks; i++)
      {
         for (j = 0; j < rows; j++)
         {
            for (k = 0; k < columns / 2; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      subPartition(assign, subMarray, subMsize, subParray, rows, columns / 2, ranks,
                   YCUT, subBounds, procBounds);
      break;

   case YCUT:
      subBounds.ymax = procBounds[parray[(rows / 2) * columns]].ymin;
      for (i = proc = 0; i < ranks; i++)
      {
         for (j = 0; j < rows / 2; j++)
         {
            for (k = 0; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      subPartition(assign, subMarray, subMsize, subParray, rows / 2, columns, ranks,
                   ZCUT, subBounds, procBounds);
      break;

   case ZCUT:
      subBounds.zmax = procBounds[parray[(ranks / 2) * rows * columns]].zmin;
      for (i = proc = 0; i < ranks / 2; i++)
      {
         for (j = 0; j < rows; j++)
         {
            for (k = 0; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      subPartition(assign, subMarray, subMsize, subParray, rows, columns, ranks / 2,
                   XCUT, subBounds, procBounds);
      break;
   }

   // Greater bisection.
   if (msize > 1)
   {
      for (i = 0; i < subMsize; i++)
      {
         subMarray[i] = marray[i + subMsize];
      }
   }
   switch (cut)
   {
   case XCUT:
      subBounds.xmin = subBounds.xmax;
      subBounds.xmax = bounds.xmax;
      for (i = proc = 0; i < ranks; i++)
      {
         for (j = 0; j < rows; j++)
         {
            for (k = columns / 2; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      subPartition(assign, subMarray, subMsize, subParray, rows, columns / 2, ranks,
                   YCUT, subBounds, procBounds);
      break;

   case YCUT:
      subBounds.ymin = subBounds.ymax;
      subBounds.ymax = bounds.ymax;
      for (i = 0, proc = 0; i < ranks; i++)
      {
         for (j = rows / 2; j < rows; j++)
         {
            for (k = 0; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      subPartition(assign, subMarray, subMsize, subParray, rows / 2, columns, ranks,
                   ZCUT, subBounds, procBounds);
      break;

   case ZCUT:
      subBounds.zmin = subBounds.zmax;
      subBounds.zmax = bounds.zmax;
      for (i = ranks / 2, proc = 0; i < ranks; i++)
      {
         for (j = 0; j < rows; j++)
         {
            for (k = 0; k < columns; k++)
            {
               subParray[proc] = parray[(i * rows * columns) + (j * columns) + k];
               proc++;
            }
         }
      }
      subPartition(assign, subMarray, subMsize, subParray, rows, columns, ranks / 2,
                   XCUT, subBounds, procBounds);
      break;
   }
   delete subMarray;
   delete subParray;
}


// Migrate objects.
void ProcessorSet::migrate()
{
   register int       i, proc;
   register OctObject *object;

   // Cull and migrate boids to appropriate octrees.
   for (proc = 0; proc < numProcs; proc++)
   {
      if (ptids[proc] == tid)
      {
         migrations[proc] = octrees[proc]->cull();
      }
      else
      {
         migrations[proc] = NULL;
      }
   }
   for (proc = 0; proc < numProcs; proc++)
   {
      while (migrations[proc] != NULL)
      {
         object           = migrations[proc];
         migrations[proc] = object->retnext;
         object->retnext  = NULL;
         for (i = 0; i < numProcs; i++)
         {
            if (i == proc)
            {
               continue;
            }
            if (object->isInside(octrees[i]))
            {
               insert(i, (Boid *)object->client);
               break;
            }
         }
#ifdef _DEBUG
         assert(i < numProcs);
#endif
         delete object;
      }
   }
}
