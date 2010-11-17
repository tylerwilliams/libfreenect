PROGRAM = glview
PROGRAM_FILES = glview.c

CC = gcc
CFLAGS	+= -g $(shell pkg-config --cflags libusb-1.0 libfreenect-1.0)
LDFLAGS	+= -g
LIBS 	+= $(shell pkg-config --libs libusb-1.0 libfreenect-1.0) 
UNAME := $(shell uname -s)

ifeq ($(UNAME), Darwin)
	CFLAGS += -framework OpenGL -framework GLUT
endif

ifeq ($(UNAME), Linux)
	LDFLAGS +=$(shell pkg-config --cflags gl)
	CFLAGS += $(shell pkg-config --libs gl)
endif


all: $(PROGRAM)

%: %.c 
	$(CC) $(PROGRAM_FILES) $(CFLAGS) $(LDFLAGS) -o $(PROGRAM) $(LIBS)

clean:
	@rm -rf $(PROGRAM)