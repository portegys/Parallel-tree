/*
 * Get time in milliseconds since the initial call.
 */

#ifdef UNIX
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#else
#include <windows.h>
#endif

typedef unsigned long long   TIME;
#define INVALID_TIME    ((unsigned long long)(-1))
TIME gettime();
