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

#ifndef __PROCESSORSET_HPP__
#define __PROCESSORSET_HPP__

#ifdef UNIX
#include <pvm3.h>
#include <pthread.h>
#endif
#include "message.h"
#include "Boid.h"
#include "octree.hpp"
#include "frustum.hpp"

#define PRECISION    100.0

class ProcessorSet
{
public:

   // Maximum boundary movement rate for load-balancing.
   static const float MAX_BOUNDARY_VELOCITY;

   // Load-balancing partitions.
   typedef enum { XCUT, YCUT, ZCUT }
   CUT;

   // Centroid.
   typedef struct Centroid
   {
      Point3D         position;
      int             load;
      struct Centroid *next;
   } CENTROID;

   // Visible object.
   typedef struct Visible
   {
      int            id;
      Point3D        position;
      Point3D        velocity;
      struct Visible *next;
   } VISIBLE;

   // Constructor.
   ProcessorSet(int dimension, float span, int numBoids,
                int *ptids, int tid, int randomSeed);

   // Destructor.
   ~ProcessorSet();

   // Make boids.
   void makeBoids(int proc, int quantity);

   // Main loop.
   void run();

   // Aim: update velocity and determine new position.
   void aim();

   // Move to new position.
   void move();

   // Report load.
   void report();

   // Report statistics.
   void stats();

   // Load-balance.
   void balance();

   // Migrate boids.
   void migrate();

   // Report ready.
   void ready();

   // Insert boid into a processor.
   bool insert(int proc, Boid *boid);

   // Search a processor.
   // Returns list of matching boids.
   Boid *search(int proc, Point3D point, float radius);

   // Search for visible local objects.
   VISIBLE *searchVisible(Frustum *frustum);

   // Serve client processors.
   void serveClient(int operation);

   // Set load-balance.
   void setLoadBalance(bool mode) { loadBalance = mode; }

   // Load-balance.
   void balance(int *parray, int rows, int columns, int ranks,
                CUT cut, Octree::BOUNDS bounds, CENTROID *centroids);

   void sortCentroids(CENTROID * *, CUT);

   // Bounds intersection.
   bool intersects(Octree::BOUNDS, Octree::BOUNDS);

   // Partition processors among machines.
   static void partition(int *assign, int numMachines, int dimension);
   static void subPartition(int *assign, int *marray, int msize, int *parray,
                            int rows, int columns, int ranks, CUT cut,
                            Octree::BOUNDS bounds, Octree::BOUNDS *procBounds);

   // Data members.
   int            dimension;
   float          span;
   float          margin;
   int            numBoids;
   int            numProcs;
   Octree         **octrees;
   OctObject      **migrations;
   int            *ptids;
   int            tid;
   Octree::BOUNDS *newBounds;
   bool           loadBalance;
   int            msgSent, msgRcv;
};
#endif
