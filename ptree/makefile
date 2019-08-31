# Build the parallel tree system.

CC = gcc

CCFLAGS = -O -DUNIX -D_DEBUG -I/opt/pvm3/include -I/opt/Mesa-2.5/include -L/opt/Mesa-2.5/lib -L/usr/X/lib

LIBS = -lnsl -lsocket -lthread -lm -lglut -lMesaGLU -lMesaGL -lX11 -lXmu -lXext

HDR = cameraGuide.hpp NamedObject.h Obstacle.h SimObject.h \
	Boid.h Vector.h frustum.hpp glutInit.h \
	message.h octree.hpp point3d.h processorSet.hpp \
	quaternion.hpp spacial.hpp

SRC = NamedObject.cpp Obstacle.cpp Boid.cpp Vector.cpp \
	frustum.cpp glutInit.cpp octree.cpp point3d.cpp processorSet.cpp

all: ptree_master ptree_slave

ptree_master: ptree_master.cpp $(HDR) $(SRC)
	$(CC) $(CCFLAGS) -o ptree_master ptree_master.cpp $(SRC) \
		/opt/pvm3/lib/$(PVM_ARCH)/libpvm3.a $(LIBS)

ptree_slave: ptree_slave.cpp $(HDR) $(SRC)
	$(CC) $(CCFLAGS) -o ptree_slave ptree_slave.cpp $(SRC) \
		/opt/pvm3/lib/$(PVM_ARCH)/libpvm3.a $(LIBS)

clean:
	/bin/rm -f ptree_master ptree_slave *.o
