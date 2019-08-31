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
 *
 * File Name : ptree_slave.cpp
 *
 * Description : PVM slave for parallel octree program.
 *
 * Date : 3/24/2003
 */

// Remove console.
#ifndef UNIX
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )
#endif

#ifndef UNIX
#include <windows.h>
#else
#include <unistd.h>
#endif
#include "processorSet.hpp"
#include <assert.h>

int main(int argc, char **argv)
{
#ifdef UNIX
   int          type, tid, dimension, numProcs, numBoids, count, random;
   float        span;
   int          *ptids;
   ProcessorSet *pset;
   int          i, j;

   char buf[200];                                 // fliber
   // Enroll In PVM.
   tid = pvm_mytid();

   // Get initialization information.
   pvm_recv(-1, 0);
   pvm_upkint(&type, 1, 1);
#ifdef _DEBUG
   assert(type == INIT);
#endif
   pvm_upkint(&dimension, 1, 1);
   pvm_upkfloat(&span, 1, 1);
   pvm_upkint(&numBoids, 1, 1);
   pvm_upkint(&count, 1, 1);
   numProcs = dimension * dimension * dimension;
   ptids    = new int[numProcs];
#ifdef _DEBUG
   assert(ptids != NULL);
#endif
   pvm_upkint(ptids, numProcs, 1);
   pvm_upkint(&random, 1, 1);

   // Create the processor set.
   Boid::setBoidCount(count);
   pset = new ProcessorSet(dimension, span, numBoids, ptids, tid, random);
#ifdef _DEBUG
   assert(pset != NULL);
#endif

   // Run.
   pset->run();

   pvm_exit();
#endif
   return(0);
}
