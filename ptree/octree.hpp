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
 * Programmers : Tom Portegys <portegys@ilstu.edu>
 *				 Kevin Greenan <kmgreen2@ilstu.edu>
 *
 * File Name :	octree.hpp
 *
 * Description : Bounded octree.
 *
 * Date : 3/24/2003
 */

#ifndef __OCTREE_HPP__
#define __OCTREE_HPP__

#include <stdio.h>
#include "point3d.h"
#include "frustum.hpp"

class OctObject;
class Octree;
class OctNode;

// Object in tree.
class OctObject
{
public:

   // Constructors.
   OctObject();
   OctObject(float x, float y, float z, void *client);
   OctObject(Point3D point, void *client);
   void init(Point3D point, void *client);
   void setClient(void *client);
   OctObject operator=(const OctObject& obj);

   // Move object.
   // Returns false if migrating out of bounds.
   bool move(float x, float y, float z);
   bool move(Point3D point);

   // Remove object from tree.
   void remove();

   // Object is inside tree?
   bool isInside(Octree *tree);

   // Object is inside node?
   bool isInside(OctNode *node);

   // Object is "close"?
   bool isClose(OctObject *object);

   // Global list link.
   OctObject *next;

   // Return list link.
   OctObject *retnext;

   // Data members.
   Point3D   position;
   OctNode   *node;
   OctObject *neighbor;
   void      *client;
};

// Octree.
class Octree
{
public:

   // Bounds.
   typedef struct
   {
      float xmin, xmax;
      float ymin, ymax;
      float zmin, zmax;
   } BOUNDS;

   // Constructors.
   Octree();
   Octree(float x, float y, float z, float span, float precision);
   Octree(Point3D center, float span, float precision);
   void init(Point3D center, float span, float precision);

   // Destructor.
   ~Octree();
   void clear();

   // Insert object.
   bool insert(OctObject *object);

   // Remove object.
   void remove(OctObject *object);

   // Search.
   // Returns list of matching objects.
   OctObject *search(float x, float y, float z, float radius);
   OctObject *search(Point3D point, float radius);

   // Search for visible objects.
   OctObject *searchVisible(Frustum *frustum);

   // Set bounds.
   void setBounds(BOUNDS bounds);

   // Cull objects outside of bounds.
   // Returns list of culled objects.
   OctObject *cull();

   // Find median point of objects.
   void findMedian();

   typedef enum { XSORT, YSORT, ZSORT }
   SORTTYPE;
   void sortObjects(SORTTYPE);

#ifdef _DEBUG
   // Audit.
   void audit();
#endif

   // Data members.
   OctNode   *root;
   Point3D   center;
   float     span;
   float     precision;
   BOUNDS    bounds;
   OctObject *objects;
   int       load;
   Point3D   median;
};

// Node.
class OctNode
{
public:

   // Constructors.
   OctNode(float x, float y, float z, float span, Octree *tree,
           OctNode *parent, OctObject *object);
   OctNode(Point3D center, float span, Octree *tree,
           OctNode *parent, OctObject *object);
   void init(Point3D center, float span, Octree *tree,
             OctNode *parent, OctObject *object);

   // Destructor.
   ~OctNode();

   // Insert object.
   bool insert(OctObject *object, bool upFlag = false);

   // Remove object.
   void remove(OctObject *object);

   // Contract node.
   void contract();

   // Move object.
   // Returns false if migrating out of bounds.
   bool move(OctObject *object);

   // Search.
   // Returns list of matching objects.
   void search(Point3D point, float radius, OctObject **list);

   // Search for visible objects.
   // Returns list of matching objects.
   void searchVisible(Frustum *frustum, OctObject **list);

#ifdef _DEBUG
   bool auditNode(Octree *);
   bool findNode(OctNode *);
#endif

   // Data members.
   Octree    *tree;
   OctObject *objects;
   OctNode   *parent;
   OctNode   *children[8];
   int       numChildren;
   Point3D   center;
   float     span;
};
#endif
