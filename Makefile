# INCLUDE = -I/usr/X11R6/include/
# LIBDIR  = -L/usr/X11R6/lib

# FLAGS = -Wall `pkg-config --cflags opencv`
# CC = g++                                  # change to gcc if using C
# CFLAGS = $(FLAGS) $(INCLUDE)
# LIBS =  -lglut -lGL -lGLU -lm `pkg-config --libs opencv` 

# app: app.o
# 	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $< $(LIBS)
	
# bilateral: bilateral.o
# 	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $< $(LIBS)

CXX = g++
CXXFLAGS = -O2 -g -Wall -fmessage-length=0 
CPPFLAGS = -I/usr/local/Cellar/opencv/2.4.9/include

OBJS = app.o

LDFLAGS = -L/usr/local/Cellar/opencv/2.4.9/lib
LDLIBS =  -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_video \
          -lopencv_features2d -lopencv_ml -lopencv_highgui -lopencv_objdetect \
          -lopencv_contrib -lopencv_legacy -lopencv_gpu

TARGET = app

.PHONY: all
all: $(TARGET)
$(TARGET):  $(OBJS)
		$(CXX) $(LDFLAGS) $^ $(LDLIBS) -o $@

.PHONY: clean
clean:
		rm -f $(OBJS) $(TARGET)