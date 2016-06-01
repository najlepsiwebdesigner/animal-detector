.DEFAULT_GOAL := app
INCLUDE = -I/usr/X11R6/include/
LIBDIR  = -L/usr/X11R6/lib

FLAGS = -Wall `pkg-config --cflags opencv`
CC = g++                                  # change to gcc if using C
CFLAGS = $(FLAGS) $(INCLUDE)
LIBS =  -lglut -lGL -lGLU -lm `pkg-config --libs opencv` 

app: app.o
	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $< $(LIBS)
	
bilateral: bilateral.o
	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $< $(LIBS)
