CC = g++
LD = g++
BINARY = jiggle

CFLAGS = -w -pipe 
OFLAGS = -c -I/usr/include
LFLAGS = $(CFLAGS) -L/usr/lib/

SOURCES = $(wildcard src/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
INCLUDE = thirdparty/eigen
CFLAGS += -I $(INCLUDE) 
CFLAGS += -framework GLUT -framework OpenGL -framework ApplicationServices 

DEBUG = no
PROFILE = no
PEDANTIC = no
OPTIMIZATION = -O3

ifeq ($(DEBUG), yes)
	CFLAGS += -g
	OPTIMIZATION = -O0
endif

ifeq ($(PROFILE), yes)
	CFLAGS += -pg
endif

CFLAGS += $(OPTIMIZATION)

all: $(BINARY)

$(BINARY): $(OBJECTS)
	$(CC) $(OBJECTS) $(CFLAGS) -o $(BINARY)

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJECTS) $(BINARY)

rebuild: clean all

.PHONY : clean
.SILENT : clean
