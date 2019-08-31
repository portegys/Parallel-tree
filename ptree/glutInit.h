// Programmer : Kevin Greenan
// Description : Class used to draw stuff
//				 Made for the 279 prog 3
// Date : 02/24/2003
//
#ifndef __GLUTINIT_H__
#define __GLUTINIT_H__

#ifndef UNIX
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

class GLUTWrapper
{
public:
   GLUTWrapper() {}
   void InitGLUT(int dm, int winX, int winY, char *name, int argcp, char **argvp);

   void RegKeyFunc(void (*func)(unsigned char key, int x, int y));
   void RegSpecialKeyFunc(void (*func)(int key, int x, int y));
   void RegMouseFunc(void (*func)(int button, int state, int x, int y));
   void RegDisplayFunc(void (*func)(void));
   void RegIdleFunc(void (*func)(void));
   void RegReshapeFunc(void (*func)(int width, int height));
   void RegVisibilityFunc(void (*func)(int));
   void RunLoop();
};
#endif
