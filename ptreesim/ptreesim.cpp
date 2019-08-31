/*
 * Programmer : Tom Portegys <portegys@ilstu.edu>
 *
 * File Name : ptreesim.cpp
 *
 * Description : Simulation of flocking boids in distributed octrees.
 *
 * Date : 3/24/2003
 */

// Remove console.
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )

#include "processorSet.hpp"
#include "cameraGuide.hpp"
#include "frustum.hpp"
#include "frameRate.hpp"
#include "glutInit.h"
#include "matrix.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <assert.h>

#ifndef _NO_NAMESPACE
using namespace std;
using namespace math;
#define STD    std
#else
#define STD
#endif

#ifndef _NO_TEMPLATE
typedef matrix<GLfloat>   Matrix;
#else
typedef matrix            Matrix;
#endif

// Pi and radians
#define M_PI          3.14159265358979323846
#define DIV_PI_180    .01745329251
#define DIV_180_PI    57.29577951

// Sizes.
#define WIN_X         500
#define WIN_Y         500
#define SPAN          15.0f
#define TRANSLATE     -45.0f

// Boids.
int NUM_BOIDS = 100;
#define BOID_SPEED_FACTOR    3.0f
int BoidDelay = 0;

// Processor set.
#define NUM_MACHINES    2
#define DIMENSION       2                         // (power of 2)
#define NUM_PROCS       (DIMENSION * DIMENSION * DIMENSION)
ProcessorSet           *Set;
ProcessorSet::PROCTYPE Ptypes[NUM_PROCS];
struct SetColor
{
   float r, g, b;
}
     *SetColors;
bool LoadBalance = false;

// Camera.
#define CAMERA_BEHIND    0.25f
CameraGuide *Guide;
GLfloat     Pitch, Yaw, Roll;
GLfloat     Speed = 0.0f;
Frustum     *frustum;
bool        DisplayOctrees = true;
Boid        *BoidGuide     = NULL;

// Rotate scene?
bool Rotate = true;

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
   "           1 : Roll right",
   "           3 : Roll left",
   "           + : Boids faster",
   "           - : Boids slower",
   "           b : Toggle random boid view",
   "           d : Toggle octree display",
   "           l : Toggle load-balancing",
   "           r : Toggle rotation",
   "           w : Full screen",
   "           q : Quit",
   NULL
};

// Modes.
enum
{
   RUN, HELP
}
UserMode;

// Frame rate management.
#define TARGET_FRAME_RATE    100.0f
FrameRate frameRate(TARGET_FRAME_RATE);

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
#define LINE_SPACE    15

// Functions.
void modeInfo(), runInfo(), helpInfo();
void setInfoProjection(), resetInfoProjection();

void renderBitmapString(GLfloat, GLfloat, void *, char *);

#define PRAND    ((float)(rand() % 1001) / 1000.0f)

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


// Draw an octree.
void drawOctree(OctNode *root, Octree::BOUNDS bounds)
{
   if (root == NULL)
   {
      return;
   }
   if (root->objects.size() > 0)
   {
      drawCube(root->center.m_x, root->center.m_y,
               root->center.m_z, root->span, bounds);
   }
   else if (root->numChildren > 0)
   {
      drawCube(root->center.m_x, root->center.m_y,
               root->center.m_z, root->span, bounds);
      if (root->children[0] != NULL)
      {
         drawOctree(root->children[0], bounds);
      }
      if (root->children[1] != NULL)
      {
         drawOctree(root->children[1], bounds);
      }
      if (root->children[2] != NULL)
      {
         drawOctree(root->children[2], bounds);
      }
      if (root->children[3] != NULL)
      {
         drawOctree(root->children[3], bounds);
      }
      if (root->children[4] != NULL)
      {
         drawOctree(root->children[4], bounds);
      }
      if (root->children[5] != NULL)
      {
         drawOctree(root->children[5], bounds);
      }
      if (root->children[6] != NULL)
      {
         drawOctree(root->children[6], bounds);
      }
      if (root->children[7] != NULL)
      {
         drawOctree(root->children[7], bounds);
      }
   }
}


// Draw a boid.
void drawBoid(ProcessorSet::VISIBLE *visible)
{
   float x, y, z;

   x = visible->position.m_x - (visible->velocity.m_x * BOID_SPEED_FACTOR);
   y = visible->position.m_y - (visible->velocity.m_y * BOID_SPEED_FACTOR);
   z = visible->position.m_z - (visible->velocity.m_z * BOID_SPEED_FACTOR);
   glColor3f(0.8f, 0.8f, 0.8f);
   glLineWidth(2.0f);
   glBegin(GL_LINES);
   glVertex3f(visible->position.m_x, visible->position.m_y, visible->position.m_z);
   glVertex3f(x, y, z);
   glEnd();
   glLineWidth(1.0f);
   glColor3f(0.9f, 0.9f, 0.9f);
   glPushMatrix();
   glTranslatef(visible->position.m_x, visible->position.m_y, visible->position.m_z);
   if (BoidGuide == NULL)
   {
      glutSolidSphere(0.1f, 10, 10);
   }
   else
   {
      glutSolidSphere(0.05f, 10, 10);
   }
   glPopMatrix();
   glColor3f(1.0f, 1.0f, 1.0f);
}


// Rotation angle.
float rotAng = 0.0;

// Draw the processor set.
void drawSet()
{
   register int                   i;
   register Octree                *tree;
   register ProcessorSet::VISIBLE *visible, *visible2;

   glPushMatrix();
   glTranslatef(0.0f, 0.0f, TRANSLATE);
   glRotatef(rotAng, 1.0f, 0.0f, 0.0f);
   glRotatef(rotAng, 0.0f, 1.0f, 0.0f);

   // Draw octrees.
   if (DisplayOctrees)
   {
      for (i = 0; i < NUM_PROCS; i++)
      {
         glColor3f(SetColors[i].r, SetColors[i].g, SetColors[i].b);
         tree = Set->octrees[i];
         drawOctree(tree->root, tree->bounds);
      }
   }
   glColor3f(1.0f, 1.0f, 1.0f);

   // Draw visible boids.
   if (frustum != NULL)
   {
      delete frustum;
   }
   frustum = new Frustum();
   visible = Set->searchVisible(frustum);
   while (visible != NULL)
   {
      drawBoid(visible);
      visible2 = visible->next;
      delete visible;
      visible = visible2;
   }

   // Draw bounds.
   glColor3f(1.0f, 1.0f, 1.0f);
   glLineWidth(2.0);
   for (i = 0; i < NUM_PROCS; i++)
   {
      tree = Set->octrees[i];
      drawBox(tree->bounds.xmin, tree->bounds.xmax, tree->bounds.ymin,
              tree->bounds.ymax, tree->bounds.zmin, tree->bounds.zmax);
   }
   glLineWidth(1.0);

   glPopMatrix();
   if (Rotate)
   {
      if (rotAng >= 360)
      {
         rotAng = 0.0f;
      }
      else
      {
         rotAng += 0.1f;
      }
   }
}


// Get world transformation matrix for boids.
void getBoidsWorldTransform(GLfloat *matrix, GLfloat translate = 0.0f)
{
   glMatrixMode(GL_MODELVIEW);
   glPushMatrix();
   glTranslatef(0.0f, 0.0f, translate);
   glRotatef(rotAng, 1.0f, 0.0f, 0.0f);
   glRotatef(rotAng, 0.0f, 1.0f, 0.0f);
   glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
   glPopMatrix();
}


// Transform boid local point to world.
void localToWorldBoid(GLfloat *local, GLfloat *world, GLfloat translate = 0.0f)
{
   int     i, j;
   GLfloat m[16];
   Matrix  x(4, 4), p(4, 1), t(4, 1);

   getBoidsWorldTransform(m, translate);
   for (i = 0; i < 4; i++)
   {
      for (j = 0; j < 4; j++)
      {
         x(i, j) = m[(j * 4) + i];
      }
   }
   p(0, 0)  = local[0];
   p(1, 0)  = local[1];
   p(2, 0)  = local[2];
   p(3, 0)  = 1.0;
   t        = x * p;
   world[0] = t(0, 0);
   world[1] = t(1, 0);
   world[2] = t(2, 0);
}


// Display.
void display()
{
   GLfloat    p[3], f[3], u[3];
   static int delayCount = 0;

   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   glColor3f(1.0, 1.0, 1.0);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   // Running?
   if (UserMode == RUN)
   {
      // Slow boids?
      delayCount++;
      if (delayCount >= BoidDelay)
      {
         delayCount = 0;

         // Update.
         Set->update(frameRate.speedFactor);
      }

      if (BoidGuide == NULL)
      {
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
      }
      else
      {
         // Boid view.
         GLfloat local[3];
         Vector  position = BoidGuide->getPosition();
         local[0] = position.x;
         local[1] = position.y;
         local[2] = position.z;
         GLfloat world[3];
         localToWorldBoid(local, world, TRANSLATE);
         p[0] = position.x = world[0];
         p[1] = position.y = world[1];
         p[2] = position.z = world[2];
         Vector velocity = BoidGuide->getVelocity();
         velocity.Normalize();
         local[0] = velocity.x;
         local[1] = velocity.y;
         local[2] = velocity.z;
         localToWorldBoid(local, world);
         velocity.x = world[0];
         velocity.y = world[1];
         velocity.z = world[2];
         velocity.Normalize();
         f[0] = position.x + velocity.x;
         f[1] = position.y + velocity.y;
         f[2] = position.z + velocity.z;
         u[0] = 1.0f;
         u[1] = 10.0f;
         u[2] = 0.0f;
         if (velocity.z != 0.0f)
         {
            u[2] = (-velocity.x - velocity.y) / velocity.z;
         }
         gluLookAt(
            p[0], p[1], p[2],
            f[0], f[1], f[2],
            u[0], u[1], u[2]);
      }

      // Draw.
      drawSet();
   }

   // Mode information.
   modeInfo();

   // Update frame rate.
   frameRate.update();

   glutSwapBuffers();
   glFlush();
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
      gluPerspective(60.0, float(w) / float(h), 0.01, 100.0);
   }
   glMatrixMode(GL_MODELVIEW);
}


// Keyboard input.
void keyboard(unsigned char key, int x, int y)
{
   std::list<Boid *>           boidList;
   std::list<Boid *>::iterator itr;

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

   // Run mode.
   case RUN:
      if (!Guide->IsTracking())
      {
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
         }
      }

      switch (key)
      {
      case '+':
         BoidDelay--;
         if (BoidDelay < 0)
         {
            BoidDelay = 0;
         }
         break;

      case '-':
         BoidDelay++;
         break;

      case 'b':
         if (BoidGuide == NULL)
         {
            Set->listBoids(boidList);
            int i = (int)boidList.size();
            if (i > 0)
            {
               i = rand() % i;
               int j = 0;
               for (itr = boidList.begin();
                    itr != boidList.end(); itr++)
               {
                  if (j == i)
                  {
                     BoidGuide = *itr;
                     Guide->SetTracking(true);
                     break;
                  }
                  j++;
               }
            }
         }
         else
         {
            BoidGuide = NULL;
            Guide->SetTracking(false);
         }
         break;

      case 'd':
         DisplayOctrees = !DisplayOctrees;
         break;

      case 'l':
         LoadBalance = !LoadBalance;
         Set->setLoadBalance(LoadBalance);
         break;

      case 'r':
         Rotate = !Rotate;
         break;

      case 'w':
         glutFullScreen();
         break;

      case '?':
         UserMode = HELP;
         break;

      case 'q':                                   // Quit.
         exit(0);
      }
      break;
   }

   // Force re-display.
   glutPostRedisplay();
}


// Special keyboard input.
void
specialKeyboard(int key, int x, int y)
{
   if (!Guide->IsTracking())
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

   // Force re-display.
   glutPostRedisplay();
}


// Window visibility.
void visible(int state)
{
   if (state == GLUT_VISIBLE)
   {
      glutIdleFunc(idle);
   }
   else
   {
      glutIdleFunc(NULL);
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
      exit(0);
   }
   glutPostRedisplay();
}


int main(int argc, char **argv)
{
   int   i, j, seed;
   int   assign[DIMENSION * DIMENSION * DIMENSION];
   float r, g, b;

#ifdef WIN32
   // Direct stdio to parent console.
   if (AttachConsole(ATTACH_PARENT_PROCESS))
   {
      freopen("CONOUT$", "w", stdout);
      freopen("CONOUT$", "w", stderr);
      freopen("CONIN$", "r", stdin);
   }
#endif

   // Get options.
   seed = time(NULL);
   for (i = 1; i < argc; i++)
   {
      if (strcmp(argv[i], "-numBoids") == 0)
      {
         i++;
         if (i >= argc)
         {
            fprintf(stderr, "Usage %s [-numBoids <number of boids>] [-randomSeed <random number seed>]\n", argv[0]);
            exit(1);
         }
         if ((NUM_BOIDS = atoi(argv[i])) < 0)
         {
            fprintf(stderr, "Usage %s [-numBoids <number of boids>] [-randomSeed <random number seed>]\n", argv[0]);
            exit(1);
         }
         continue;
      }

      if (strcmp(argv[i], "-randomSeed") == 0)
      {
         i++;
         if (i >= argc)
         {
            fprintf(stderr, "Usage %s [-numBoids <number of boids>] [-randomSeed <random number seed>]\n", argv[0]);
            exit(1);
         }
         seed = atoi(argv[i]);
         continue;
      }

      fprintf(stderr, "Usage %s [-numBoids <number of boids>] [-randomSeed <random number seed>]\n", argv[0]);
      exit(1);
   }

   GLUTWrapper glObj;

   glObj.InitGLUT(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH, WIN_X, WIN_Y, "Parallel Octree Simulation", argc, argv);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   if (WIN_Y != 0)
   {
      gluPerspective(60.0, float(WIN_X / WIN_Y), 0.01, 100.0);
   }
   gluLookAt(0.0f, 0.0f, 5.0f, 0.0, 0.0, 0.0, 0.0, 1.0, 0.);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   glObj.RegDisplayFunc(display);
   glObj.RegIdleFunc(idle);
   glObj.RegKeyFunc(keyboard);
   glObj.RegSpecialKeyFunc(specialKeyboard);
   glObj.RegReshapeFunc(reshape);
   glObj.RegVisibilityFunc(visible);
   glutCreateMenu(menu);
   glutAddMenuEntry("Full screen", 1);
   glutAddMenuEntry("Exit", 2);
   glutAttachMenu(GLUT_RIGHT_BUTTON);

   // Create processor set.
   for (i = 0; i < NUM_PROCS; i++)
   {
      Ptypes[i] = ProcessorSet::LOCAL;
   }
   Set = new ProcessorSet(DIMENSION, SPAN, NUM_BOIDS, Ptypes, seed);
   assert(Set != NULL);

   // Partition processors among machines and color-code by machine.
   SetColors = new struct SetColor[NUM_PROCS];
   assert(SetColors != NULL);
   ProcessorSet::partition(assign, NUM_MACHINES, DIMENSION);
   for (i = 0; i < NUM_MACHINES; i++)
   {
      r = (PRAND * 0.5f) + 0.5f;
      g = (PRAND * 0.5f) + 0.5f;
      b = (PRAND * 0.5f) + 0.5f;
      for (j = 0; j < NUM_PROCS; j++)
      {
         if (assign[j] == i)
         {
            SetColors[j].r = r;
            SetColors[j].g = g;
            SetColors[j].b = b;
         }
      }
   }

   // Create camera guide and frustum.
   Guide = new CameraGuide();
   assert(Guide != NULL);
   Pitch = -90.0;
   Guide->SetPitch(Pitch);
   frustum = new Frustum();

   // Set user mode.
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

   UserMode == RUN ? runInfo() : helpInfo();

   resetInfoProjection();
}


// Run information.
void runInfo()
{
   renderBitmapString(5, 10, FONT, "? for help");
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
