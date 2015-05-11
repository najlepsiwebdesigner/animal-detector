# INCLUDE = -I/usr/X11R6/include/
# LIBDIR  = -L/usr/X11R6/lib

# FLAGS = -Wall `pkg-config --cflags opencv`
# CC = g++                                  # change to gcc if using C
# CFLAGS = $(FLAGS) $(INCLUDE)
# LIBS =  -lglut -lGL -lGLU -lm `pkg-config --libs opencv` 

# peto: peto.o
# 	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $< $(LIBS)
	
# bilateral: bilateral.o
# 	$(CC) $(CFLAGS) -o $@ $(LIBDIR) $< $(LIBS)

CXX = g++
CXXFLAGS = -O2 -g -Wall -fmessage-length=0 
CPPFLAGS = -I/usr/local/Cellar/opencv/2.4.9/include

PETO_OBJS = peto.o
FERO_OBJS = fero.o
CANNY_OBJS = canny.o
FEAT_OBJS = feat.o

LDFLAGS = -L/usr/local/Cellar/opencv/2.4.9/lib
LDLIBS =  -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_video \
          -lopencv_features2d -lopencv_ml -lopencv_highgui -lopencv_objdetect \
          -lopencv_contrib -lopencv_legacy -lopencv_gpu -lopencv_nonfree -lopencv_flann

PETO_TARGET = apps/peto
FERO_TARGET = apps/fero
CANNY_TARGET = apps/canny
FEAT_TARGET = apps/feat

.PHONY: peto
peto: $(PETO_TARGET)
$(PETO_TARGET):  $(PETO_OBJS)
		$(CXX) $(LDFLAGS) $^ $(LDLIBS) -o $@

.PHONY: clean
clean:
		rm -f $(PETO_OBJS) $(PETO_TARGET)
		rm -f $(FERO_OBJS) $(FERO_TARGET)

.PHONY: fero
fero: $(FERO_TARGET)
$(FERO_TARGET): $(FERO_OBJS)
		$(CXX) $(LXFLAGS) $^ $(LDLIBS) -o $@		

		
.PHONY: canny
canny: $(CANNY_TARGET)
$(CANNY_TARGET): $(CANNY_OBJS)
		$(CXX) $(LXFLAGS) $^ $(LDLIBS) -o $@		

		
.PHONY: feat
feat: $(FEAT_TARGET)
$(FEAT_TARGET): $(FEAT_OBJS)
		$(CXX) $(LqXFLAGS) $^ $(LDLIBS) -o $@		


.PHONY: run-feat
run-feat:
		./apps/feat features/1.jpg features/2.jpg



.PHONY: run
run:
		./apps/peto ${ARGS}


.PHONY: runone
runone:
		./apps/peto -vid ./apps/1.mpeg -img ./apps/1.jpg