CC = g++

CFLAGS = -Wall -ggdb -std=c++0x -pg -pthread

OOGL_STATIC_LIB = ../lib/OOGL.a
OOGL_INCLUDE = ../include
EIGEN_INCLUDE = /usr/include/eigen3

LIBS = $(OOGL_STATIC_LIB) -lX11 -lXrandr -lGL -lm

INCLUDES = -I $(OOGL_INCLUDE) -I $(EIGEN_INCLUDE)

all: main.cpp ../bin
	$(CC) $(CFLAGS) main.cpp -o main $(INCLUDES) $(LIBS)
	mv main ../bin
