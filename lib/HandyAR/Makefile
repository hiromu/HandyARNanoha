INCLUDE_DIR = ../../include
LIB_DIR = ..

OBJS = ColorDistribution.o ColorHistogram.o FingerTip.o FingertipPoseEstimation.o FingertipTracker.o HandRegion.o MixGaussian.o PoseEstimation.o
CXXFLAGS = -I$(INCLUDE_DIR) #-DUSE_GAUSSIAN_MODEL

all: libHandyAR.a

libHandyAR.a : $(OBJS)
	ar rc $(LIB_DIR)/$@ *.o

clean:
	rm -f $(OBJS)
	rm -f $(LIB_DIR)/libHandyAR.a
