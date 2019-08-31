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
 * File Name : octree.cpp
 *
 * Description : Bounded octree.
 *
 * Date : 3/24/2003
 */

#include "octree.hpp"
#include <assert.h>

// Constructors.
OctObject::OctObject()
{
   Point3D point;

   point.set(0.0f, 0.0f, 0.0f);
   init(point, NULL);
}


OctObject::OctObject(float x, float y, float z, void *client)
{
   Point3D point;

   point.set(x, y, z);
   init(point, client);
}


OctObject::OctObject(Point3D point, void *client)
{
   init(point, client);
}


void OctObject::init(Point3D point, void *client)
{
   position     = point;
   node         = NULL;
   next         = neighbor = NULL;
   this->client = client;
}


void OctObject::setClient(void *client)
{
   this->client = client;
}


// Move object.
bool OctObject::move(float x, float y, float z)
{
   Point3D point;

   point.set(x, y, z);
   return(move(point));
}


bool OctObject::move(Point3D point)
{
   position = point;
   if (node != NULL)
   {
      return(node->move(this));
   }
   return(false);
}


// Remove object from tree.
void OctObject::remove()
{
   if (node != NULL)
   {
      node->tree->remove(this);
   }
}


// Object is inside tree?
bool OctObject::isInside(Octree *tree)
{
   if (position.m_x < (tree->center.m_x - tree->span))
   {
      return(false);
   }
   if (position.m_x >= (tree->center.m_x + tree->span))
   {
      return(false);
   }
   if (position.m_y < (tree->center.m_y - tree->span))
   {
      return(false);
   }
   if (position.m_y >= (tree->center.m_y + tree->span))
   {
      return(false);
   }
   if (position.m_z < (tree->center.m_z - tree->span))
   {
      return(false);
   }
   if (position.m_z >= (tree->center.m_z + tree->span))
   {
      return(false);
   }
   if (position.m_x < tree->bounds.xmin)
   {
      return(false);
   }
   if (position.m_x >= tree->bounds.xmax)
   {
      return(false);
   }
   if (position.m_y < tree->bounds.ymin)
   {
      return(false);
   }
   if (position.m_y >= tree->bounds.ymax)
   {
      return(false);
   }
   if (position.m_z < tree->bounds.zmin)
   {
      return(false);
   }
   if (position.m_z >= tree->bounds.zmax)
   {
      return(false);
   }
   return(true);
}


// Object is inside node?
bool OctObject::isInside(OctNode *node)
{
   if (position.m_x < (node->center.m_x - node->span))
   {
      return(false);
   }
   if (position.m_x >= (node->center.m_x + node->span))
   {
      return(false);
   }
   if (position.m_y < (node->center.m_y - node->span))
   {
      return(false);
   }
   if (position.m_y >= (node->center.m_y + node->span))
   {
      return(false);
   }
   if (position.m_z < (node->center.m_z - node->span))
   {
      return(false);
   }
   if (position.m_z >= (node->center.m_z + node->span))
   {
      return(false);
   }
   if (node->tree == NULL)
   {
      return(true);
   }
   if (position.m_x < node->tree->bounds.xmin)
   {
      return(false);
   }
   if (position.m_x >= node->tree->bounds.xmax)
   {
      return(false);
   }
   if (position.m_y < node->tree->bounds.ymin)
   {
      return(false);
   }
   if (position.m_y >= node->tree->bounds.ymax)
   {
      return(false);
   }
   if (position.m_z < node->tree->bounds.zmin)
   {
      return(false);
   }
   if (position.m_z >= node->tree->bounds.zmax)
   {
      return(false);
   }
   return(true);
}


// Object is "close"?
bool OctObject::isClose(OctObject *object)
{
#ifdef _DEBUG
   assert(node != NULL);
#endif
   if ((int)(position.DistSquare(object->position) * node->tree->precision) == 0)
   {
      return(true);
   }
   else
   {
      return(false);
   }
}


// Constructors.
Octree::Octree()
{
   Point3D center;

   center.set(0.0f, 0.0f, 0.0f);
   init(center, 1.0f, 0.1f);
}


Octree::Octree(float x, float y, float z, float span, float precision)
{
   Point3D center;

   center.set(x, y, z);
   init(center, span, precision);
}


Octree::Octree(Point3D center, float span, float precision)
{
   init(center, span, precision);
}


void Octree::init(Point3D center, float span, float precision)
{
   root            = NULL;
   this->center    = center;
   this->span      = span;
   this->precision = precision;
   bounds.xmin     = center.m_x - span;
   bounds.xmax     = center.m_x + span;
   bounds.ymin     = center.m_y - span;
   bounds.ymax     = center.m_y + span;
   bounds.zmin     = center.m_z - span;
   bounds.zmax     = center.m_z + span;
   objects         = NULL;
   load            = 0;
}


// Destructor.
Octree::~Octree()
{
   clear();
}


void Octree::clear()
{
   if (root != NULL)
   {
      delete root;
   }
   root    = NULL;
   objects = NULL;
}


// Insert object.
bool Octree::insert(OctObject *object)
{
   bool ret;

#ifdef _DEBUG
   assert(object != NULL);
#endif
   if (!object->isInside(this))
   {
      return(false);
   }

   // Insert into tree.
   if (root == NULL)
   {
      root = new OctNode(center, span, this, NULL, object);
      ret  = true;
   }
   else
   {
      ret = root->insert(object);
   }
   if (ret == false)
   {
      return(false);
   }

   // Insert into object list.
   object->next = objects;
   objects      = object;
   load++;

   return(true);
}


// Remove object.
void Octree::remove(OctObject *object)
{
   register OctObject *o, *o2;

#ifdef _DEBUG
   assert(object != NULL);
#endif
   // Remove from tree.
   if (object->node != NULL)
   {
      object->node->remove(object);
   }

   // Remove from object list.
   for (o = objects, o2 = NULL; o != NULL; o2 = o, o = o->next)
   {
      if (o == object)
      {
         break;
      }
   }
#ifdef _DEBUG
   assert(o != NULL);
#endif
   if (o2 == NULL)
   {
      objects = object->next;
   }
   else
   {
      o2->next = object->next;
   }
   object->next = NULL;
   load--;
}


// Search.
// Returns list of matching objects.
OctObject *Octree::search(float x, float y, float z, float radius)
{
   Point3D point;

   point.set(x, y, z);
   return(search(point, radius));
}


OctObject *Octree::search(Point3D point, float radius)
{
   OctObject *list = NULL;

   if (root != NULL)
   {
      root->search(point, radius, &list);
   }
   return(list);
}


// Search for visible objects.
// Returns list of matching objects.
OctObject *Octree::searchVisible(Frustum *frustum)
{
   OctObject *list = NULL;

   if (root != NULL)
   {
      root->searchVisible(frustum, &list);
   }
   return(list);
}


// Set bounds.
void Octree::setBounds(BOUNDS bounds)
{
   this->bounds = bounds;
}


// Cull objects outside of bounds.
// Returns list of culled objects.
OctObject *Octree::cull()
{
   register OctObject *list = NULL;
   register OctObject *o, *o2;

   for (o = objects, o2 = NULL; o != NULL; )
   {
      if (!o->isInside(this))
      {
         load--;
         if (o->node != NULL)
         {
            o->node->remove(o);
         }
         if (o2 == NULL)
         {
            objects = o->next;
         }
         else
         {
            o2->next = o->next;
         }
         o->retnext = list;
         list       = o;
         o          = o->next;
      }
      else
      {
         o2 = o;
         o  = o->next;
      }
   }
   return(list);
}


// Find median point of objects.
void Octree::findMedian()
{
   register OctObject *o, *o2;
   float              d, m;

   if (load == 0)
   {
      median.m_x = (bounds.xmax - bounds.xmin) / 2.0f;
      median.m_y = (bounds.ymax - bounds.ymin) / 2.0f;
      median.m_z = (bounds.zmax - bounds.zmin) / 2.0f;
      return;
   }

   // Get X median.
   sortObjects(XSORT);
   d = 0.0f;
   m = ((float)load - 1.0f) / 2.0f;
   for (o = objects, o2 = NULL; o != NULL; o2 = o, o = o->next)
   {
      if (d >= m)
      {
         break;
      }
      d += 1.0f;
   }
   if (d > m)
   {
      median.m_x = (o2->position.m_x + o->position.m_x) / 2.0f;
   }
   else
   {
      median.m_x = o->position.m_x;
   }

   // Get Y median.
   sortObjects(YSORT);
   d = 0.0f;
   for (o = objects, o2 = NULL; o != NULL; o2 = o, o = o->next)
   {
      if (d >= m)
      {
         break;
      }
      d += 1.0f;
   }
   if (d > m)
   {
      median.m_y = (o2->position.m_y + o->position.m_y) / 2.0f;
   }
   else
   {
      median.m_y = o->position.m_y;
   }

   // Get Z median.
   sortObjects(ZSORT);
   d = 0.0f;
   for (o = objects, o2 = NULL; o != NULL; o2 = o, o = o->next)
   {
      if (d >= m)
      {
         break;
      }
      d += 1.0f;
   }
   if (d > m)
   {
      median.m_z = (o2->position.m_z + o->position.m_z) / 2.0f;
   }
   else
   {
      median.m_z = o->position.m_z;
   }
}


// Sort objects by dimension.
void Octree::sortObjects(SORTTYPE type)
{
   register OctObject *head, *o, *o2, *o3;

   head = NULL;
   while (objects != NULL)
   {
      for (o = head, o2 = NULL; o != NULL; o2 = o, o = o->next)
      {
         if ((type == XSORT) && (o->position.m_x >= objects->position.m_x))
         {
            break;
         }
         if ((type == YSORT) && (o->position.m_y >= objects->position.m_y))
         {
            break;
         }
         if ((type == ZSORT) && (o->position.m_z >= objects->position.m_z))
         {
            break;
         }
      }
      o3       = objects;
      objects  = objects->next;
      o3->next = o;
      if (o2 == NULL)
      {
         head = o3;
      }
      else
      {
         o2->next = o3;
      }
   }
   objects = head;
}


#ifdef _DEBUG
// Audit.
void Octree::audit()
{
   register int       count;
   register OctObject *object, *o;

   for (object = objects, count = 0; object != NULL; object = object->next)
   {
      assert(object->node != NULL);
      assert(object->node->tree != NULL);
      assert(object->node->tree == this);
      assert(root != NULL && root->findNode(object->node));
      for (o = object->node->objects; o != NULL; o = o->neighbor)
      {
         if (o == object)
         {
            break;
         }
      }
      assert(o != NULL);
      count++;
   }
   assert(count == load);
   if (root != NULL)
   {
      assert(root->auditNode(this));
   }
}


bool OctNode::findNode(OctNode *node)
{
   register int i;

   if (node == this)
   {
      return(true);
   }

   for (i = 0; i < 8; i++)
   {
      if ((children[i] != NULL) && children[i]->findNode(node))
      {
         return(true);
      }
   }
   return(false);
}


bool OctNode::auditNode(Octree *tree)
{
   register int       i;
   register OctObject *o, *o2;

   for (o = objects; o != NULL; o = o->neighbor)
   {
      assert(o->node == this);
      assert(o->isInside(this));
      for (o2 = tree->objects; o2 != NULL; o2 = o2->next)
      {
         if (o2 == o)
         {
            break;
         }
      }
      if (o2 == NULL)
      {
         return(false);
      }
   }

   for (i = 0; i < 8; i++)
   {
      if ((children[i] != NULL) && !children[i]->auditNode(tree))
      {
         return(false);
      }
   }
   return(true);
}


#endif

// Constructors.
OctNode::OctNode(float x, float y, float z, float span,
                 Octree *tree, OctNode *parent, OctObject *object)
{
   Point3D center;

   center.set(x, y, z);
   init(center, span, tree, parent, object);
}


OctNode::OctNode(Point3D center, float span, Octree *tree,
                 OctNode *parent, OctObject *object)
{
   init(center, span, tree, parent, object);
}


void OctNode::init(Point3D center, float span, Octree *tree,
                   OctNode *parent, OctObject *object)
{
   int i;

   this->center = center;
   this->span   = span;
   this->tree   = tree;
   this->parent = parent;
   for (i = 0; i < 8; i++)
   {
      children[i] = NULL;
   }
   numChildren = 0;
   objects     = object;
   if (object != NULL)
   {
      object->node = this;
   }
}


// Destructor.
OctNode::~OctNode()
{
   register int       i;
   register OctObject *o, *o2;

   o       = objects;
   objects = NULL;
   while (o != NULL)
   {
      o2          = o->neighbor;
      o->neighbor = NULL;
      delete o;
      o = o2;
   }

   for (i = 0; i < 8; i++)
   {
      if (children[i] == NULL)
      {
         continue;
      }
      delete children[i];
      children[i] = NULL;
   }
}


// Insert object.
bool OctNode::insert(OctObject *object, bool upFlag)
{
   float     span2;
   OctObject *o, *o2;

#ifdef _DEBUG
   assert(object != NULL);
#endif
   // Object is inside this node?
   if (upFlag && !object->isInside(this))
   {
      // Try parent.
      if (parent != NULL)
      {
         return(parent->insert(object, upFlag));
      }
      return(false);
   }

   // Insert into this node?
   if (((objects == NULL) && (numChildren == 0)) ||
       ((objects != NULL) && objects->isClose(object)))
   {
      object->neighbor = objects;
      objects          = object;
      object->node     = this;
      return(true);
   }

   // Insert object into child.
   span2 = span / 2.0f;
   if (object->position.m_z < center.m_z)
   {
      if (object->position.m_x < center.m_x)
      {
         if (object->position.m_y < center.m_y)
         {
            if (children[0] == NULL)
            {
               children[0] = new OctNode(center.m_x - span2, center.m_y - span2,
                                         center.m_z - span2, span2, tree, this, object);
               assert(children[0] != NULL);
               numChildren++;
            }
            else
            {
               children[0]->insert(object);
            }
         }
         else
         {
            if (children[1] == NULL)
            {
               children[1] = new OctNode(center.m_x - span2, center.m_y + span2,
                                         center.m_z - span2, span2, tree, this, object);
               assert(children[1] != NULL);
               numChildren++;
            }
            else
            {
               children[1]->insert(object);
            }
         }
      }
      else
      {
         if (object->position.m_y < center.m_y)
         {
            if (children[2] == NULL)
            {
               children[2] = new OctNode(center.m_x + span2, center.m_y - span2,
                                         center.m_z - span2, span2, tree, this, object);
               assert(children[2] != NULL);
               numChildren++;
            }
            else
            {
               children[2]->insert(object);
            }
         }
         else
         {
            if (children[3] == NULL)
            {
               children[3] = new OctNode(center.m_x + span2, center.m_y + span2,
                                         center.m_z - span2, span2, tree, this, object);
               assert(children[3] != NULL);
               numChildren++;
            }
            else
            {
               children[3]->insert(object);
            }
         }
      }
   }
   else
   {
      if (object->position.m_x < center.m_x)
      {
         if (object->position.m_y < center.m_y)
         {
            if (children[4] == NULL)
            {
               children[4] = new OctNode(center.m_x - span2, center.m_y - span2,
                                         center.m_z + span2, span2, tree, this, object);
               assert(children[4] != NULL);
               numChildren++;
            }
            else
            {
               children[4]->insert(object);
            }
         }
         else
         {
            if (children[5] == NULL)
            {
               children[5] = new OctNode(center.m_x - span2, center.m_y + span2,
                                         center.m_z + span2, span2, tree, this, object);
               assert(children[5] != NULL);
               numChildren++;
            }
            else
            {
               children[5]->insert(object);
            }
         }
      }
      else
      {
         if (object->position.m_y < center.m_y)
         {
            if (children[6] == NULL)
            {
               children[6] = new OctNode(center.m_x + span2, center.m_y - span2,
                                         center.m_z + span2, span2, tree, this, object);
               assert(children[6] != NULL);
               numChildren++;
            }
            else
            {
               children[6]->insert(object);
            }
         }
         else
         {
            if (children[7] == NULL)
            {
               children[7] = new OctNode(center.m_x + span2, center.m_y + span2,
                                         center.m_z + span2, span2, tree, this, object);
               assert(children[7] != NULL);
               numChildren++;
            }
            else
            {
               children[7]->insert(object);
            }
         }
      }
   }

   // Re-insert existing objects.
   o       = objects;
   objects = NULL;
   while (o != NULL)
   {
      o2          = o->neighbor;
      o->neighbor = NULL;
      insert(o);
      o = o2;
   }

   return(true);
}


// Remove object.
void OctNode::remove(OctObject *object)
{
   register OctObject *o, *o2;

#ifdef _DEBUG
   assert(object != NULL);
#endif
   // Unlink from object list.
   for (o = objects, o2 = NULL; o != NULL; o2 = o, o = o->neighbor)
   {
      if (o == object)
      {
         break;
      }
   }
#ifdef _DEBUG
   assert(o != NULL);
#endif
   if (o2 == NULL)
   {
      objects = o->neighbor;
   }
   else
   {
      o2->neighbor = o->neighbor;
   }
   o->node     = NULL;
   o->neighbor = NULL;

   // Contract parent.
   if (parent != NULL)
   {
      parent->contract();
   }
}


// Contract node.
void OctNode::contract()
{
   register int       i, j;
   register OctObject *object;

   // Delete empty children.
   for (i = 0, j = -1; i < 8; i++)
   {
      if (children[i] == NULL)
      {
         continue;
      }
      if (children[i]->numChildren == 0)
      {
         if (children[i]->objects == NULL)
         {
            delete children[i];
            children[i] = NULL;
            numChildren--;
         }
         else
         {
            j = i;
         }
      }
   }

   // Contract parent?
   if (numChildren == 0)
   {
      if (parent != NULL)
      {
         parent->contract();
      }
   }
   else if (numChildren == 1)
   {
      // Bring up single child's objects?
      if (j != -1)
      {
         objects = children[j]->objects;
         children[j]->objects = NULL;
         for (object = objects; object != NULL; object = object->neighbor)
         {
            object->node = this;
         }
         delete children[j];
         children[j] = NULL;
         numChildren--;
         if (parent != NULL)
         {
            parent->contract();
         }
      }
   }
}


// Move object.
bool OctNode::move(OctObject *object)
{
   register OctObject *o, *o2;
   bool               ret;

#ifdef _DEBUG
   assert(object != NULL);
#endif
   // Find object.
   for (o = objects, o2 = NULL; o != NULL; o2 = o, o = o->neighbor)
   {
      if (o == object)
      {
         break;
      }
   }
#ifdef _DEBUG
   assert(o != NULL);
#endif

   // Object remains in node?
   if (object->isInside(this))
   {
      return(true);
   }

   // Remove from node.
   if (o2 == NULL)
   {
      objects = o->neighbor;
   }
   else
   {
      o2->neighbor = o->neighbor;
   }
   o->node     = NULL;
   o->neighbor = NULL;

   // Insert into parent.
   ret = false;
   if ((parent != NULL) && parent->insert(o, true))
   {
      ret = true;
   }

   // Contract parent?
   if (objects == NULL)
   {
      if (parent != NULL)
      {
         parent->contract();
      }
   }

   return(ret);
}


// Search.
// Returns list of matching objects.
void OctNode::search(Point3D point, float radius, OctObject **list)
{
   register int       i;
   register OctObject *object;
   register OctNode   *child;
   float              r2;

   // Get squared search radius.
   r2 = radius * radius;

   // Check for objects within search radius.
   for (object = objects; object != NULL; object = object->neighbor)
   {
      if (object->position.DistSquare(point) <= r2)
      {
         object->retnext = *list;
         *list           = object;
      }
   }

   // Search matching children.
   for (i = 0; i < 8; i++)
   {
      if ((child = children[i]) == NULL)
      {
         continue;
      }
      if ((point.m_x + radius) < (child->center.m_x - child->span))
      {
         continue;
      }
      if ((point.m_x - radius) > (child->center.m_x + child->span))
      {
         continue;
      }
      if ((point.m_y + radius) < (child->center.m_y - child->span))
      {
         continue;
      }
      if ((point.m_y - radius) > (child->center.m_y + child->span))
      {
         continue;
      }
      if ((point.m_z + radius) < (child->center.m_z - child->span))
      {
         continue;
      }
      if ((point.m_z - radius) > (child->center.m_z + child->span))
      {
         continue;
      }
      children[i]->search(point, radius, list);
   }
}


// Search for visible objects.
// Returns list of matching objects.
void OctNode::searchVisible(Frustum *frustum, OctObject **list)
{
   register int       i;
   register OctObject *object;
   register OctNode   *child;
   float              xmin, xmax, ymin, ymax, zmin, zmax;

   // Node intersects frustum?
   xmin = center.m_x - span;
   if (xmin < tree->bounds.xmin)
   {
      xmin = tree->bounds.xmin;
   }
   xmax = center.m_x + span;
   if (xmax > tree->bounds.xmax)
   {
      xmax = tree->bounds.xmax;
   }
   ymin = center.m_y - span;
   if (ymin < tree->bounds.ymin)
   {
      ymin = tree->bounds.ymin;
   }
   ymax = center.m_y + span;
   if (ymax > tree->bounds.ymax)
   {
      ymax = tree->bounds.ymax;
   }
   zmin = center.m_z - span;
   if (zmin < tree->bounds.zmin)
   {
      zmin = tree->bounds.zmin;
   }
   zmax = center.m_z + span;
   if (zmax > tree->bounds.zmax)
   {
      zmax = tree->bounds.zmax;
   }
   if (!frustum->intersects(xmin, xmax, ymin, ymax, zmin, zmax))
   {
      return;
   }

   // Check for objects within frustum.
   for (object = objects; object != NULL; object = object->neighbor)
   {
      if (frustum->isInside(object->position))
      {
         object->retnext = *list;
         *list           = object;
      }
   }

   // Search children.
   for (i = 0; i < 8; i++)
   {
      if ((child = children[i]) == NULL)
      {
         continue;
      }
      children[i]->searchVisible(frustum, list);
   }
}
