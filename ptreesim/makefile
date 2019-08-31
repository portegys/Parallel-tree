# ptreesim makefile.

CC = gcc

CCFLAGS = -DUNIX -O3
LINKLIBS = -lglut -lGLU -lGL -lm -lstdc++

all: ptreesim

ptreesim: *.h *.hpp *.cpp
	$(CC) $(CCFLAGS) -o ptreesim *.cpp $(LINKLIBS)

clean:
	/bin/rm -f *.o




