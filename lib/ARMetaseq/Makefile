INCLUDE_DIR = ../../include
LIB_DIR = ..

OBJS = OpenGLExtension.o ARMetaseq.o 
CFLAGS = -I$(INCLUDE_DIR)

all: libARMetaseq.a

libARMetaseq.a : $(OBJS)
	ar rc $(LIB_DIR)/$@ *.o

clean:
	rm -f $(OBJS)
	rm -f $(LIB_DIR)/libARMetaseq.a
