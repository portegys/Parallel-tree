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

#include "Boid.h"
#include "octree.hpp"
#include "frustum.hpp"
#include <list>

#define PRECISION    100.0

class ProcessorSet
{
public:

   // Processor type.
   typedef enum { LOCAL, REMOTE }
   PROCTYPE;

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

   // Visible element.
   typedef struct Visible
   {
      int            id;
      Point3D        position;
      Point3D        velocity;
      struct Visible *next;
   } VISIBLE;

   // Constructor.
   ProcessorSet(int dimension, float span, int numBoids,
                PROCTYPE *ptypes, int randomSeed);

   // Destructor.
   ~ProcessorSet();

   // Make boids.
   void makeBoids(int proc, int quantity);

   // Update.
   void update(float simRate = 1.0f);

   // Insert boid into a processor.
   bool insert(int proc, Boid *boid);

   // List boids.
   void listBoids(std::list<Boid *>& boidList);

   // Search a processor.
   // Returns list of matching boids.
   void search(int proc, Point3D point, float radius, std::list<Boid>& boidList);

   // Search for visible local objects.
   VISIBLE *searchVisible(Frustum *frustum);

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
   int    dimension;
   float  span;
   float  margin;
   int    numBoids;
   int    numProcs;
   Octree **octrees;
   std::list<OctObject *> *migrations;
   PROCTYPE               *ptypes;
   Octree::BOUNDS         *newBounds;
   bool loadBalance;
};
#endif
