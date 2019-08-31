/*
 * Programmer : Tom Portegys <portegys@ilstu.edu>
 * File Name : ProcessorSet.cpp
 *
 * Description : Flocking boids (alife creatures) tracked by octrees.
 *               The octrees are distributed among processors.
 *
 * Date : 3/13/2003
 */

#include "processorSet.hpp"
#include <stdlib.h>
#include <assert.h>
#include <time.h>

#define PRAND    ((float)(rand() % 1001) / 1000.0f)

// Maximum boundary movement rate for load-balancing.
const float ProcessorSet::MAX_BOUNDARY_VELOCITY = 0.1f;

// Constructor.
ProcessorSet::ProcessorSet(int dimension, float span, int numBoids,
                           PROCTYPE *ptypes, int randomSeed)
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
   margin          = span / 20.0f;
   Boid::setSpan(span);
   this->numBoids = numBoids;
   numProcs       = dimension * dimension * dimension;
   this->ptypes   = new PROCTYPE[numProcs];
#ifdef _DEBUG
   assert(this->ptypes != NULL);
#endif
   for (i = 0; i < numProcs; i++)
   {
      this->ptypes[i] = ptypes[i];
   }

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
      if (this->ptypes[j] == LOCAL)
      {
         assign[j]++;
         i++;
      }
   }

   octrees = new Octree *[numProcs];
#ifdef _DEBUG
   assert(octrees != NULL);
#endif
   migrations = new std::list<OctObject *>[numProcs];
#ifdef _DEBUG
   assert(migrations != NULL);
#endif
   size = (span * 2.0f) / dimension;
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

            // Set oct bounds.
            bounds.xmin = ((float)k * size) - span;
            bounds.xmax = bounds.xmin + size;
            bounds.ymin = ((float)j * size) - span;
            bounds.ymax = bounds.ymin + size;
            bounds.zmin = ((float)i * size) - span;
            bounds.zmax = bounds.zmin + size;
            octrees[proc]->setBounds(bounds);

            // Create boids.
            if (this->ptypes[proc] == LOCAL)
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
   delete ptypes;
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
      velocity.SetMagnitude(PRAND * MAX_VELOCITY);
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
      object = new OctObject((float)position.x, (float)position.y,
                             (float)position.z, (void *)boid);
#ifdef _DEBUG
      assert(object != NULL);
#endif

      // Insert into octree.
      octrees[proc]->insert(object);
   }
}


// Update.
void ProcessorSet::update(float simRate)
{
   register int       i, proc;
   register OctObject *object;

   std::list<OctObject *>           tmpList;
   std::list<OctObject *>::iterator itr;
   Octree::BOUNDS    bounds;
   register CENTROID *centroids, *centroid;
   int               *parray;
   std::list<Boid>   boidList;
   register Boid     *boid;
   Vector            position;

   // Load-balance?
   if (loadBalance)
   {
      bounds.xmax = bounds.ymax = bounds.zmax = span;
      bounds.xmin = bounds.ymin = bounds.zmin = -span;
      centroids   = NULL;
      for (proc = 0; proc < numProcs; proc++)
      {
         centroid = new CENTROID;
#ifdef _DEBUG
         assert(centroid != NULL);
#endif
         centroid->next = centroids;
         centroids      = centroid;
         centroid->load = octrees[proc]->load;
#ifdef _DEBUG
         assert(ptypes[proc] == LOCAL);
#endif
         octrees[proc]->findMedian();
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

      // Cull and migrate boids to revised octrees.
      for (proc = 0; proc < numProcs; proc++)
      {
         if (ptypes[proc] == LOCAL)
         {
            octrees[proc]->cull(migrations[proc]);
         }
      }
      for (proc = 0; proc < numProcs; proc++)
      {
         for (itr = migrations[proc].begin();
              itr != migrations[proc].end(); itr++)
         {
            object = *itr;
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
         migrations[proc].clear();
      }
   }

   // Update phase 1: update boid velocity and acceleration and determine new position.
   for (proc = 0; proc < numProcs; proc++)
   {
      // Update local objects.
      if (ptypes[proc] != LOCAL)
      {
         continue;
      }

      for (itr = octrees[proc]->objects.begin();
           itr != octrees[proc]->objects.end(); itr++)
      {
         object = *itr;

         // Do cross-processor search.
         boidList.clear();
         float range = (float)Boid::visibilityRange;
         bounds.xmin = object->position.m_x - range;
         bounds.xmax = object->position.m_x + range;
         bounds.ymin = object->position.m_y - range;
         bounds.ymax = object->position.m_y + range;
         bounds.zmin = object->position.m_z - range;
         bounds.zmax = object->position.m_z + range;
         for (i = 0; i < numProcs; i++)
         {
            // Search intersects processor space?
            if (intersects(octrees[i]->bounds, bounds))
            {
               // Accumulate search results.
               search(i, object->position, range, boidList);
            }
         }

         // Update boid based on search results.
         boid = (Boid *)object->client;
         boid->aim(boidList, simRate);
      }
   }

   // Update phase 2: Move and migrate boids.
   for (proc = 0; proc < numProcs; proc++)
   {
      // Update local objects.
      if (ptypes[proc] != LOCAL)
      {
         continue;
      }

      tmpList.clear();
      for (itr = octrees[proc]->objects.begin();
           itr != octrees[proc]->objects.end(); itr++)
      {
         object = *itr;
         boid   = (Boid *)object->client;
         boid->move();
         position = boid->getPosition();
         if (!object->move((float)position.x, (float)position.y, (float)position.z))
         {
            // Boid migrating processors.
            octrees[proc]->load--;
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
         }
         else
         {
            tmpList.push_back(object);
         }
      }
      octrees[proc]->objects.clear();
      for (itr = tmpList.begin(); itr != tmpList.end(); itr++)
      {
         object = *itr;
         octrees[proc]->objects.push_back(object);
      }
   }
}


// Insert boid into a processor.
bool ProcessorSet::insert(int proc, Boid *boid)
{
   Vector    position = boid->getPosition();
   OctObject *object  = new OctObject((float)position.x,
                                      (float)position.y, (float)position.z, (void *)boid);

   if (ptypes[proc] == LOCAL)
   {
      // Local insert.
      return(octrees[proc]->insert(object));
   }
   else
   {
      // Remote insert.
      assert(false);
      return(false);
   }
}


// List all boids.
void ProcessorSet::listBoids(std::list<Boid *>& boidList)
{
   std::list<OctObject *>::iterator itr;
   register OctObject               *object;
   register Boid *boid;

   boidList.clear();
   for (int proc = 0; proc < numProcs; proc++)
   {
      for (itr = octrees[proc]->objects.begin();
           itr != octrees[proc]->objects.end(); itr++)
      {
         object = *itr;
         boid   = (Boid *)object->client;
         boidList.push_front(boid);
      }
   }
}


// Search a processor.
// Returns list of matching boids.
void ProcessorSet::search(int proc, Point3D point, float radius, std::list<Boid>& boidList)
{
   register OctObject *object;

   std::list<OctObject *>           searchList;
   std::list<OctObject *>::iterator itr;
   register Boid *boid;

   if (ptypes[proc] == LOCAL)
   {
      // Local search.
      octrees[proc]->search(point, radius, searchList);
      for (itr = searchList.begin();
           itr != searchList.end(); itr++)
      {
         object = *itr;
         boid   = (Boid *)object->client;
         boidList.push_front(*boid);
      }
   }
   else
   {
      // Remote search.
      assert(false);
   }
}


// Search for visible local objects.
ProcessorSet::VISIBLE *ProcessorSet::searchVisible(Frustum *frustum)
{
   register int       proc;
   register OctObject *object;

   std::list<OctObject *>           searchList;
   std::list<OctObject *>::iterator itr;
   register Boid    *boid;
   register VISIBLE *visible, *visibleList;
   Vector           velocity;

   visibleList = NULL;
   for (proc = 0; proc < numProcs; proc++)
   {
      if (ptypes[proc] == LOCAL)
      {
         octrees[proc]->searchVisible(frustum, searchList);
         for (itr = searchList.begin();
              itr != searchList.end(); itr++)
         {
            object  = *itr;
            visible = new VISIBLE;
#ifdef _DEBUG
            assert(visible != NULL);
#endif
            boid                  = (Boid *)object->client;
            visible->id           = boid->getBoidNumber();
            visible->position     = object->position;
            velocity              = boid->getVelocity();
            visible->velocity.m_x = velocity.x;
            visible->velocity.m_y = velocity.y;
            visible->velocity.m_z = velocity.z;
            visible->next         = visibleList;
            visibleList           = visible;
         }
      }
   }
   return(visibleList);
}


// Load-balance octrees.
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

   // Number of machines should be one or an even number.
   assert(numMachines == 1 || (numMachines % 2) == 0);

   // Dimension must be power of 2.
   for (i = 1; i < dimension; i = 2 * i)
   {
   }
   assert(i == dimension);

   assert(i == dimension);
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
