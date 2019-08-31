// Programmer : Kevin Greenan
// Description : Class used to draw stuff
//				 Made for the 279 prog 3
// Date : 02/24/2003
//

#include "glutInit.h"

void GLUTWrapper::InitGLUT(int dm, int winX, int winY, char *name, int argcp, char **argvp)
{
   glutInit(&argcp, argvp);
   glutInitDisplayMode(dm);
   glutInitWindowSize(winX, winY);
   glutCreateWindow(name);
   glClearColor(0.0, 0.0, 0.0, 0.0);
   glShadeModel(GL_FLAT);
}


void GLUTWrapper::RegKeyFunc(void (*func)(unsigned char key, int x, int y))
{
   glutKeyboardFunc(func);
}


void GLUTWrapper::RegSpecialKeyFunc(void (*func)(int key, int x, int y))
{
   glutSpecialFunc(func);
}


void GLUTWrapper::RegMouseFunc(void (*func)(int button, int state, int x, int y))
{
   glutMouseFunc(func);
}


void GLUTWrapper::RegDisplayFunc(void (*func)(void))
{
   glutDisplayFunc(func);
}


void GLUTWrapper::RegIdleFunc(void (*func)(void))
{
   glutIdleFunc(func);
}


void GLUTWrapper::RegReshapeFunc(void (*func)(int width, int height))
{
   glutReshapeFunc(func);
}


void GLUTWrapper::RegVisibilityFunc(void (*func)(int state))
{
   glutVisibilityFunc(func);
}


void GLUTWrapper::RunLoop()
{
   glutMainLoop();
}
