BASE = asst1

all: $(BASE)

OS := $(shell uname -s)

ifeq ($(OS), Linux) # Science Center Linux Boxes
  CPPFLAGS = -I/home/l/i/lib175/usr/glew/include
  LDFLAGS += -L/home/l/i/lib175/usr/glew/lib -L/usr/X11R6/lib
  LIBS += -lGL -lGLU -lglut
endif

ifeq ($(OS), Darwin) # Assume OS X
  CPPFLAGS += -D__MAC__
  # added for Mavericks: 09/08/2014
  CPPFLAGS += -I/usr/include
  LDFLAGS += -framework GLUT -framework OpenGL
endif

ifdef OPT 
  #turn on optimization
  CXXFLAGS += -O2
else 
  #turn on debugging
  CXXFLAGS += -g
endif

CXX = g++ 

OBJ = $(BASE).o ppm.o glsupport.o

$(BASE): $(OBJ)
	# FIXME: this no longer works on Mavericks
	#$(LINK.cpp) -o $@ $^ $(LIBS) -lGLEW 
	# temporary solution
	$(LINK.cpp) -o $@ $^ $(LIBS) /usr/lib/libGLEW.a

clean:
	rm -f $(OBJ) $(BASE)
