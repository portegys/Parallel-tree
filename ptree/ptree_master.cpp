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
 * File Name : ptree_master.cpp
 *
 * Description : PVM master for parallel octree program.
 *
 * Environment: MY_PVM set to directory containing ptree_slave and hostfile.
 *
 * Date : 3/30/2003
 */

// Remove console.
#ifndef UNIX
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )
#endif

#include "processorSet.hpp"
#include "cameraGuide.hpp"
#include "frustum.hpp"
#include "glutInit.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#ifdef UNIX
#include <unistd.h>
#include <errno.h>
#endif

// Sizes.
#define WIN_X                500
#define WIN_Y                500
#define SPAN                 15.0f

// Boids.
#define NUM_BOIDS            50
#define BOID_SPEED_FACTOR    3.0f

// Update thread.
#ifdef UNIX
pthread_t       UpdateThread;
pthread_cond_t  UpdateCond;
pthread_mutex_t UpdateMutex;
#endif

// Random number seed.
int RandomSeed;

// Slave machines.
#define DEFAULT_NUM_MACHINES    2
int numMachines;
#define SLAVE_NAME              "ptree_slave"
int *Tids;

// Per machine message counters.
bool GetStats = false;
int  *MsgSent;
int  *MsgRcv;
int  *Load;
void getStats();

#define STATS_FILE    "stats.txt"
FILE *Statsfp;

// Processors.
#define DIMENSION    2                            // (power of 2)
#define NUM_PROCS    (DIMENSION * DIMENSION * DIMENSION)
int          Tid, Ptids[NUM_PROCS];
ProcessorSet *ProxySet;
bool         LoadBalance = false;

// Camera.
#define GUIDE_Z          100.0f
#define CAMERA_BEHIND    0.25f
CameraGuide *Guide;
GLfloat     Pitch, Yaw, Roll;
GLfloat     Speed = 0.0f;
Frustum     *frustum;

// Visible objects.
ProcessorSet::VISIBLE *VisibleList;
#ifdef UNIX
pthread_mutex_t VisibleMutex;
#endif

// Control information.
char *ControlInfo[] =
{
   "           ? : Control help. Twice to print help to standard output",
   "           f : Faster",
   "           s : Slower",
   "  Left arrow : Yaw left",
   " Right arrow : Yaw right",
   "    Up arrow : Pitch up",
   "  Down arrow : Pitch down",
   "           b : Run blind",
   "           1 : Roll right",
   "           3 : Roll left",
   "           l : Toggle load-balancing",
   "           c : Toggle statistics collecting",
   "           q : Quit",
   NULL
};

// Modes.
enum
{
   RUN, BLIND, HELP
}
UserMode;

/*
 *  Available fonts:
 *  GLUT_BITMAP_8_BY_13
 *  GLUT_BITMAP_9_BY_15
 *  GLUT_BITMAP_TIMES_ROMAN_10
 *  GLUT_BITMAP_TIMES_ROMAN_24
 *  GLUT_BITMAP_HELVETICA_10
 *  GLUT_BITMAP_HELVETICA_12
 *  GLUT_BITMAP_HELVETICA_18
 */
#define FONT          GLUT_BITMAP_9_BY_15
#define BIG_FONT      GLUT_BITMAP_TIMES_ROMAN_24
#define LINE_SPACE    10

// Functions.
void modeInfo(), runInfo(), blindInfo(), helpInfo();
void setInfoProjection(), resetInfoProjection();

void renderBitmapString(GLfloat, GLfloat, void *, char *);

#define PRAND       ((float)(rand() % 1001) / 1000.0f)
#define PATHSIZE    100

// Draw a box.
void drawBox(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
{
   glBegin(GL_LINE_LOOP);
   glVertex3f(xmax, ymax, zmax);
   glVertex3f(xmin, ymax, zmax);
   glVertex3f(xmin, ymin, zmax);
   glVertex3f(xmax, ymin, zmax);
   glEnd();

   glBegin(GL_LINE_LOOP);
   glVertex3f(xmax, ymax, zmin);
   glVertex3f(xmin, ymax, zmin);
   glVertex3f(xmin, ymin, zmin);
   glVertex3f(xmax, ymin, zmin);
   glEnd();

   glBegin(GL_LINES);
   glVertex3f(xmax, ymax, zmax);
   glVertex3f(xmax, ymax, zmin);
   glVertex3f(xmin, ymax, zmax);
   glVertex3f(xmin, ymax, zmin);
   glVertex3f(xmax, ymin, zmax);
   glVertex3f(xmax, ymin, zmin);
   glVertex3f(xmin, ymin, zmax);
   glVertex3f(xmin, ymin, zmin);
   glEnd();
}


// Draw a bounded cube.
void drawCube(float x, float y, float z, float span, Octree::BOUNDS bounds)
{
   float xmin, xmax, ymin, ymax, zmin, zmax;

   xmin = x - span;
   if (xmin < bounds.xmin)
   {
      xmin = bounds.xmin;
   }
   xmax = x + span;
   if (xmax > bounds.xmax)
   {
      xmax = bounds.xmax;
   }
   ymin = y - span;
   if (ymin < bounds.ymin)
   {
      ymin = bounds.ymin;
   }
   ymax = y + span;
   if (ymax > bounds.ymax)
   {
      ymax = bounds.ymax;
   }
   zmin = z - span;
   if (zmin < bounds.zmin)
   {
      zmin = bounds.zmin;
   }
   zmax = z + span;
   if (zmax > bounds.zmax)
   {
      zmax = bounds.zmax;
   }

   drawBox(xmin, xmax, ymin, ymax, zmin, zmax);
}


// Draw a boid.
void drawBoid(ProcessorSet::VISIBLE *visible)
{
   float x, y, z;

   glPushMatrix();
   glTranslatef(visible->position.m_x, visible->position.m_y, visible->position.m_z);
   glutSolidSphere(0.1f, 10, 10);
   glPopMatrix();
   x = visible->position.m_x - (visible->velocity.m_x * BOID_SPEED_FACTOR);
   y = visible->position.m_y - (visible->velocity.m_y * BOID_SPEED_FACTOR);
   z = visible->position.m_z - (visible->velocity.m_z * BOID_SPEED_FACTOR);
   glBegin(GL_LINES);
   glBegin(GL_LINES);
   glVertex3f(visible->position.m_x, visible->position.m_y, visible->position.m_z);
   glVertex3f(x, y, z);
   glEnd();
}


// Draw the processor set.
void drawSet()
{
   register int                   i;
   register Octree                *tree;
   register ProcessorSet::VISIBLE *visible;
   GLfloat p[3], f[3], u[3];

   // Camera follows guide.
   Guide->Update();
   Guide->GetPosition(p);
   Guide->GetForward(f);
   Guide->GetUp(u);
   gluLookAt(p[0] + (u[0] * CAMERA_BEHIND),
             p[1] + (u[1] * CAMERA_BEHIND),
             p[2] + (u[2] * CAMERA_BEHIND),
             p[0], p[1], p[2],
             f[0], f[1], f[2]);

   // Draw visible boids.
#ifdef UNIX
   pthread_mutex_lock(&VisibleMutex);
#endif
   visible = VisibleList;
   while (visible != NULL)
   {
      drawBoid(visible);
      visible = visible->next;
   }

   // Draw bounds.
   glLineWidth(2.0);
   for (i = 0; i < NUM_PROCS; i++)
   {
      tree = ProxySet->octrees[i];
      drawBox(tree->bounds.xmin, tree->bounds.xmax, tree->bounds.ymin,
              tree->bounds.ymax, tree->bounds.zmin, tree->bounds.zmax);
   }
   glLineWidth(1.0);
#ifdef UNIX
   pthread_mutex_unlock(&VisibleMutex);
#endif
}


// Display.
void display()
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glColor3f(1.0, 1.0, 1.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   if (UserMode == RUN)
   {
      drawSet();
   }

   // Mode information.
   modeInfo();

   glutSwapBuffers();
   glFlush();

   // Allow update to run.
#ifdef UNIX
   pthread_cond_signal(&UpdateCond);
#endif
}


// Idle.
void idle()
{
   glutPostRedisplay();
}


// Reshape window.
void reshape(int w, int h)
{
   glViewport(0, 0, w, h);
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   if (h != 0)
   {
      gluPerspective(60.0, float(w) / float(h), 2.0, -100.0);
   }
   glMatrixMode(GL_MODELVIEW);
}


// Keyboard input.
void keyboard(unsigned char key, int x, int y)
{
   switch (UserMode)
   {
   // Hit any key to continue from help.
   case HELP:
      if (key == '?')                             // Twice prints control help.
      {
         for (int i = 0; ControlInfo[i] != NULL; i++)
         {
            printf("%s\n", ControlInfo[i]);
         }
         break;
      }
      UserMode = RUN;
      break;

   // Running blind.
   case BLIND:
      if (key == '?')
      {
         UserMode = HELP;
      }
      else
      {
         UserMode = RUN;
      }
      break;

   // Run mode.
   case RUN:

      switch (key)
      {
      case 'f':
         Speed += 0.01f;
         Guide->SetSpeed(Speed);
         break;

      case 's':
         Speed -= 0.01f;
         if (Speed < 0.0f)
         {
            Speed = 0.0f;
         }
         Guide->SetSpeed(Speed);
         break;

      case '1':
         Roll += 1.0f;
         Guide->SetRoll(Roll);
         break;

      case '3':
         Roll -= 1.0f;
         Guide->SetRoll(Roll);
         break;

      case 'b':
         UserMode = BLIND;
         break;

      case 'l':
         LoadBalance = !LoadBalance;
         break;

      case 'c':
         GetStats = !GetStats;
         break;

      case '?':
         UserMode = HELP;
         break;

      case 'q':                                   // Quit.
#ifdef UNIX
         pvm_halt();
#endif
         exit(0);
         break;
      }
      break;
   }
}


// Special keyboard input.
void
specialKeyboard(int key, int x, int y)
{
   switch (key)
   {
   case GLUT_KEY_UP:
      Pitch += 1.0f;
      Guide->SetPitch(Pitch);
      break;

   case GLUT_KEY_DOWN:
      Pitch -= 1.0f;
      Guide->SetPitch(Pitch);
      break;

   case GLUT_KEY_RIGHT:
      Yaw += 1.0f;
      Guide->SetYaw(Yaw);
      break;

   case GLUT_KEY_LEFT:
      Yaw -= 1.0f;
      Guide->SetYaw(Yaw);
      break;
   }
}


// Menu function.
void menu(int value)
{
   switch (value)
   {
   case 1:
      glutFullScreen();
      break;

   case 2:
#ifdef UNIX
      pvm_halt();
#endif
      exit(0);
      break;
   }
}


// Get statistics.
void getStats()
{
#ifdef UNIX
   register int mach;
   int          operation, sent, rcv, load;

   // Request statistics.
   pvm_initsend(PvmDataDefault);
   operation = STATS;
   pvm_pkint(&operation, 1, 1);
   pvm_mcast(Tids, numMachines, 0);

   // Get results.
   for (mach = 0; mach < numMachines; mach++)
   {
      pvm_recv(Tids[mach], 0);
      pvm_upkint(&operation, 1, 1);
#ifdef _DEBUG
      assert(operation == STATS_RESULT);
#endif
      pvm_upkint(&sent, 1, 1);
      MsgSent[mach] += sent;
      pvm_upkint(&rcv, 1, 1);
      MsgRcv[mach] += rcv;
      pvm_upkint(&load, 1, 1);
      Load[mach] = load;
      fprintf(Statsfp, "%d %d %d %d\n", mach, load, sent, rcv);
      fflush(Statsfp);
   }
#endif
}


// Get visible objects.
void getVisible()
{
   GLfloat p[3], f[3], u[3];
   register ProcessorSet::VISIBLE *newVisibleList, *visible;

#ifdef UNIX
   register int i, j, proc;
   int          operation, result, packets, size;
#endif

   // Camera follows guide.
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   Guide->Update();
   Guide->GetPosition(p);
   Guide->GetForward(f);
   Guide->GetUp(u);
   gluLookAt(p[0] + (u[0] * CAMERA_BEHIND),
             p[1] + (u[1] * CAMERA_BEHIND),
             p[2] + (u[2] * CAMERA_BEHIND),
             p[0], p[1], p[2],
             f[0], f[1], f[2]);

   if (frustum != NULL)
   {
      delete frustum;
   }
   frustum = new Frustum();

   // Request objects.
   newVisibleList = NULL;
#ifdef UNIX
   for (proc = 0; proc < NUM_PROCS; proc++)
   {
      pvm_initsend(PvmDataDefault);
      operation = VIEW;
      pvm_pkint(&operation, 1, 1);
      pvm_pkint(&proc, 1, 1);
      for (i = 0; i < 6; i++)
      {
         pvm_pkfloat(&frustum->planes[i].a, 1, 1);
         pvm_pkfloat(&frustum->planes[i].b, 1, 1);
         pvm_pkfloat(&frustum->planes[i].c, 1, 1);
         pvm_pkfloat(&frustum->planes[i].d, 1, 1);
      }
      pvm_send(Ptids[proc], 0);

      // Get search results.
      pvm_recv(Ptids[proc], 0);
      pvm_upkint(&operation, 1, 1);
#ifdef _DEBUG
      assert(operation == VIEW_RESULT);
#endif
      pvm_upkint(&packets, 1, 1);
      for (i = 0; i < packets; i++)
      {
         if (i > 0)
         {
            pvm_recv(Ptids[proc], 0);
            pvm_upkint(&operation, 1, 1);
#ifdef _DEBUG
            assert(operation == VIEW_RESULT);
#endif
         }
         pvm_upkint(&size, 1, 1);
         for (j = 0; j < size; j++)
         {
            visible = new ProcessorSet::VISIBLE;
#ifdef _DEBUG
            assert(visible != NULL);
#endif
            pvm_upkint(&(visible->id), 1, 1);
            pvm_upkfloat(&(visible->position.m_x), 1, 1);
            pvm_upkfloat(&(visible->position.m_y), 1, 1);
            pvm_upkfloat(&(visible->position.m_z), 1, 1);
            pvm_upkfloat(&(visible->velocity.m_x), 1, 1);
            pvm_upkfloat(&(visible->velocity.m_y), 1, 1);
            pvm_upkfloat(&(visible->velocity.m_z), 1, 1);
            visible->next  = newVisibleList;
            newVisibleList = visible;
         }
      }
   }
#endif

   // Replace visible list.
#ifdef UNIX
   pthread_mutex_lock(&VisibleMutex);
#endif
   while (VisibleList != NULL)
   {
      visible     = VisibleList;
      VisibleList = VisibleList->next;
      delete visible;
   }
   VisibleList = newVisibleList;
#ifdef UNIX
   pthread_mutex_unlock(&VisibleMutex);
#endif
}


// Gather ready messages from slaves.
void gatherReady()
{
   int count, operation, proc;

#ifdef UNIX
   // Collect ready messages.
   for (count = 0; count < NUM_PROCS; count++)
   {
      pvm_recv(-1, 0);
      pvm_upkint(&operation, 1, 1);
#ifdef _DEBUG
      assert(operation == READY);
#endif
      pvm_upkint(&proc, 1, 1);
   }
#endif
}


// Update.
#ifdef UNIX
void *update(void *arg)
{
   int   i, mach, proc, balance, count;
   int   operation, dimension;
   float span;
   char  *pvmdir, hostfile[PATHSIZE + 1];
   char  machineName[PATHSIZE + 1], slavePath[PATHSIZE + 1];
   bool  useHostfile;
   int   argc;
   char  *argv[1];
   FILE  *fp;
   int   *machAssign, *boidAssign;

   // Get environment.
   pvmdir = getenv("MY_PVM");
   if ((pvmdir == NULL) || (*pvmdir == '\0'))
   {
      fprintf(stderr, "MY_PVM not set\n");
      exit(1);
   }
   argc        = 0;
   argv[0]     = NULL;
   numMachines = DEFAULT_NUM_MACHINES;
   useHostfile = false;
   sprintf(hostfile, "%s/hostfile", pvmdir);
   if (access(hostfile, F_OK) == 0)
   {
      argv[0] = hostfile;
      argc    = 1;
      if ((fp = fopen(hostfile, "r")) == NULL)
      {
         fprintf(stderr, "Cannot open hostfile %s\n", hostfile);
         exit(1);
      }
      for (numMachines = 0; fscanf(fp, "%s", machineName) == 1; numMachines++)
      {
      }
      fclose(fp);
      if ((numMachines < 1) || ((numMachines > 1) && ((numMachines % 2) != 0)))
      {
         fprintf(stderr, "Number of machines must be one or even\n");
         exit(1);
      }
      useHostfile = true;
   }

   // Clear message counters.
   MsgSent = new int[numMachines];
   MsgRcv  = new int[numMachines];
   Load    = new int[numMachines];
   for (i = 0; i < numMachines; i++)
   {
      MsgSent[i] = MsgRcv[i] = Load[i] = 0;
   }

   // Randomly assign boids to machines.
   boidAssign = new int[numMachines];
   for (mach = 0; mach < numMachines; mach++)
   {
      boidAssign[mach] = 0;
   }
   for (i = 0; i < NUM_BOIDS; i++)
   {
      mach = rand() % numMachines;
      boidAssign[mach]++;
   }

   // Partition processors among slave machines.
   machAssign = new int[NUM_PROCS];
   ProcessorSet::partition(machAssign, numMachines, DIMENSION);

   // Start PVM
   if (pvm_start_pvmd(argc, argv, 1) != 0)
   {
      fprintf(stderr, "Cannot start PVM daemon\n");
      exit(1);
   }
   Tid = pvm_mytid();

   // Start slaves.
   sprintf(slavePath, "%s/%s", pvmdir, SLAVE_NAME);
   Tids = new int[numMachines];
   if (useHostfile)
   {
      if ((fp = fopen(hostfile, "r")) == NULL)
      {
         fprintf(stderr, "Cannot open hostfile %s\n", hostfile);
         pvm_halt();
         exit(1);
      }
   }
   for (mach = 0; mach < numMachines; mach++)
   {
      strcpy(machineName, "");
      if (useHostfile && (fscanf(fp, "%s", machineName) != 1))
      {
         fprintf(stderr, "Error reading hostfile %s\n", hostfile);
         pvm_halt();
         exit(1);
      }
      if (pvm_spawn(slavePath, (char **)0, 0, machineName, 1, &Tids[mach]) != 1)
      {
         fprintf(stderr, "Cannot spawn slave. Error code = %d\n", Tids[i]);
         pvm_halt();
         exit(1);
      }
   }
   if (useHostfile)
   {
      fclose(fp);
   }
   for (mach = 0; mach < numMachines; mach++)
   {
      for (proc = 0; proc < NUM_PROCS; proc++)
      {
         if (machAssign[proc] == mach)
         {
            Ptids[proc] = Tids[mach];
         }
      }
   }

   // Open stats file.
   if ((Statsfp = fopen(STATS_FILE, "w")) == NULL)
   {
      fprintf(stderr, "Cannot open statistics file %s\n", STATS_FILE);
      pvm_halt();
      exit(1);
   }

   // Send initialization messages to slaves.
   operation = INIT;
   dimension = DIMENSION;
   span      = SPAN;
   for (mach = count = 0; mach < numMachines; count += boidAssign[mach], mach++)
   {
      pvm_initsend(PvmDataDefault);
      pvm_pkint(&operation, 1, 1);
      pvm_pkint(&dimension, 1, 1);
      pvm_pkfloat(&span, 1, 1);
      pvm_pkint(&boidAssign[mach], 1, 1);
      pvm_pkint(&count, 1, 1);
      pvm_pkint(Ptids, NUM_PROCS, 1);
      pvm_pkint(&RandomSeed, 1, 1);
      pvm_send(Tids[mach], 0);
   }

   // Update loop.
   while (true)
   {
      // Wait for display to run?
      if (UserMode == RUN)
      {
         pthread_cond_wait(&UpdateCond, &UpdateMutex);
      }

      // Send aim message to update velocity and prepare to move.
      operation = AIM;
      pvm_initsend(PvmDataDefault);
      pvm_pkint(&operation, 1, 1);
      pvm_mcast(Ptids, NUM_PROCS, 0);
      gatherReady();

      // Send move message to update position.
      operation = MOVE;
      pvm_initsend(PvmDataDefault);
      pvm_pkint(&operation, 1, 1);
      pvm_mcast(Ptids, NUM_PROCS, 0);
      gatherReady();

      // Load-balance?
      if (LoadBalance)
      {
         balance = 1;
      }
      else
      {
         balance = 0;
      }
      if (balance)
      {
         // Get load-balance status.
         operation = REPORT;
         pvm_initsend(PvmDataDefault);
         pvm_pkint(&operation, 1, 1);
         pvm_mcast(Ptids, NUM_PROCS, 0);

         // Collect report results.
         for (count = 0; count < NUM_PROCS; count++)
         {
            pvm_recv(-1, 0);
            pvm_upkint(&operation, 1, 1);
#ifdef _DEBUG
            assert(operation == REPORT_RESULT);
#endif
            pvm_upkint(&proc, 1, 1);
            pvm_upkint(&(ProxySet->octrees[proc]->load), 1, 1);
            pvm_upkfloat(&(ProxySet->octrees[proc]->median.m_x), 1, 1);
            pvm_upkfloat(&(ProxySet->octrees[proc]->median.m_y), 1, 1);
            pvm_upkfloat(&(ProxySet->octrees[proc]->median.m_z), 1, 1);
         }

         // Load-balance.
#ifdef UNIX
         pthread_mutex_lock(&VisibleMutex);
#endif
         ProxySet->balance();
#ifdef UNIX
         pthread_mutex_unlock(&VisibleMutex);
#endif

         // Distribute load-balanced bounds.
         operation = BALANCE;
         pvm_initsend(PvmDataDefault);
         pvm_pkint(&operation, 1, 1);
         for (proc = 0; proc < NUM_PROCS; proc++)
         {
            pvm_pkfloat(&(ProxySet->octrees[proc]->bounds.xmin), 1, 1);
            pvm_pkfloat(&(ProxySet->octrees[proc]->bounds.xmax), 1, 1);
            pvm_pkfloat(&(ProxySet->octrees[proc]->bounds.ymin), 1, 1);
            pvm_pkfloat(&(ProxySet->octrees[proc]->bounds.ymax), 1, 1);
            pvm_pkfloat(&(ProxySet->octrees[proc]->bounds.zmin), 1, 1);
            pvm_pkfloat(&(ProxySet->octrees[proc]->bounds.zmax), 1, 1);
         }
         pvm_mcast(Ptids, NUM_PROCS, 0);
         gatherReady();

         // Migrate boids.
         operation = MIGRATE;
         pvm_initsend(PvmDataDefault);
         pvm_pkint(&operation, 1, 1);
         pvm_mcast(Ptids, NUM_PROCS, 0);
         gatherReady();
      }

      // Get statistics?
      if (GetStats)
      {
         getStats();
      }

      // Get visible objects?
      if (UserMode == RUN)
      {
         getVisible();
      }
   }
}


#endif

int main(int argc, char **argv)
{
   GLUTWrapper glObj;
   GLfloat     v[3];
   int         proc, dummyTids[NUM_PROCS];

   // Set random seed.
   if (argc == 2)
   {
      RandomSeed = atoi(argv[1]);
   }
   else if (argc == 1)
   {
      RandomSeed = time(NULL);
   }
   else
   {
      fprintf(stderr, "Usage %s [random number seed]\n", argv[0]);
      exit(1);
   }
   srand(RandomSeed);

   // Create display.
   glObj.InitGLUT(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH, WIN_X, WIN_Y, "Parallel Octree", argc, argv);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   if (WIN_Y != 0)
   {
      gluPerspective(60.0, float(WIN_X / WIN_Y), 0.0, 100.0);
   }
   gluLookAt(0.0f, 0.0f, GUIDE_Z + CAMERA_BEHIND, 0.0, 0.0, 0.0, 0.0, 1.0, 0.);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   glObj.RegDisplayFunc(display);
   glObj.RegIdleFunc(idle);
   glObj.RegKeyFunc(keyboard);
   glObj.RegSpecialKeyFunc(specialKeyboard);
   glObj.RegReshapeFunc(reshape);
   glObj.RegReshapeFunc(reshape);
   glutCreateMenu(menu);
   glutAddMenuEntry("Full screen", 1);
   glutAddMenuEntry("Exit", 2);
   glutAttachMenu(GLUT_RIGHT_BUTTON);

   // Create camera guide and frustum.
   Guide = new CameraGuide();
   assert(Guide != NULL);
   Pitch = -90.0;
   Guide->SetPitch(Pitch);
   v[0] = v[1] = 0.0f;
   v[2] = GUIDE_Z;
   Guide->SetPosition(v);
   frustum = new Frustum();

   // Message counters created in update thread.
   MsgSent = MsgRcv = NULL;

   // Create visible object list.
   VisibleList = NULL;
#ifdef UNIX
   pthread_mutex_init(&VisibleMutex, NULL);
#endif

   // Create proxy processor set.
   for (proc = 0; proc < NUM_PROCS; proc++)
   {
      dummyTids[proc] = 0;
   }
   ProxySet = new ProcessorSet(DIMENSION, SPAN, 0, dummyTids, 0, RandomSeed);
#ifdef _DEBUG
   assert(ProxySet != NULL);
#endif

   // Create update thread.
#ifdef UNIX
   if (pthread_create(&UpdateThread, NULL, update, (void *)0) != 0)
   {
      fprintf(stderr, "%s: cannot create update thread, errno=%d\n", argv[0], errno);
      exit(1);
   }
   pthread_cond_init(&UpdateCond, NULL);
   pthread_mutex_init(&UpdateMutex, NULL);
#endif

   // Run the display.
   UserMode = RUN;
   glObj.RunLoop();

   return(0);
}


// Display mode information.
void modeInfo()
{
   glColor3f(1.0, 1.0, 1.0);
   glDisable(GL_BLEND);
   glDisable(GL_TEXTURE_2D);
   glDisable(GL_LIGHTING);
   glLineWidth(2.0);

   setInfoProjection();

   switch (UserMode)
   {
   case RUN:
      runInfo();
      break;

   case HELP:
      helpInfo();
      break;

   case BLIND:
      blindInfo();
      break;
   }

   resetInfoProjection();
}


// Run information.
void runInfo()
{
   int  i, j;
   char buf[100];

   renderBitmapString(5, 10, FONT, "? for help");
   if (!GetStats)
   {
      return;
   }
   for (i = j = 0; i < numMachines; i++)
   {
      j += MsgSent[i];
   }
   sprintf(buf, "Total msg sent: %d", j);
   renderBitmapString(5, WIN_Y - 20, FONT, buf);
   for (i = j = 0; i < numMachines; i++)
   {
      j += MsgRcv[i];
   }
   sprintf(buf, "Total msg rcv: %d", j);
   renderBitmapString(WIN_X - 150, WIN_Y - 20, FONT, buf);
}


// Help for controls.
void helpInfo()
{
   int i, v;

   // Clear the screen.
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   v = 10;
   renderBitmapString(5, v, FONT, "Controls:");
   v += (2 * LINE_SPACE);
   for (i = 0; ControlInfo[i] != NULL; i++)
   {
      renderBitmapString(5, v, FONT, ControlInfo[i]);
      v += LINE_SPACE;
   }
}


// Running blind message.
void blindInfo()
{
   // Clear the screen.
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   renderBitmapString((WIN_X / 2) - 25, WIN_Y / 2, BIG_FONT, "Running blind");
}


void setInfoProjection()
{
   glMatrixMode(GL_PROJECTION);
   glPushMatrix();
   glLoadIdentity();
   gluOrtho2D(0, WIN_X, 0, WIN_Y);

   // Invert the y axis, down is positive.
   glScalef(1, -1, 1);

   // Move the origin from the bottom left corner to the upper left corner.
   glTranslatef(0, -WIN_Y, 0);

   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glLoadIdentity();
}


void resetInfoProjection()
{
   glPopMatrix();
   glMatrixMode(GL_PROJECTION);
   glPopMatrix();
   glMatrixMode(GL_MODELVIEW);
}


// Print string on screen at specified location.
void renderBitmapString(GLfloat x, GLfloat y, void *font, char *string)
{
   char *c;

   glRasterPos2f(x, y);
   for (c = string; *c != '\0'; c++)
   {
      glutBitmapCharacter(font, *c);
   }
}
