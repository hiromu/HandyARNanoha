BIN_DIR = ../bin
INCLUDE_DIR = ../include
LIB_DIR = ../lib

OBJS = capture.o main.o
LIBS = -lHandyAR -lARMetaseq -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_legacy -lopencv_calib3d -lopencv_video -lglut -lGLU -lpng

CXXFLAGS = -I$(INCLUDE_DIR)
LDFLAGS = -L$(LIB_DIR)

all: AR

AR: $(OBJS)
	c++ -o $(BIN_DIR)/$@ $(OBJS) $(LDFLAGS) $(LIBS)

clean:
	rm -f $(OBJS)
	rm -f $(BIN_DIR)/AR
