# Edit the following variables as needed
Mavlink = /home/patipan/Mavlink

EXTLIB		= -L$(HDF_INSTALL)/lib
CC          = arm-linux-gnueabihf-g++
CFLAGS      = 
LIB         = 

INCLUDE		= -I$(Mavlink)/include/common
LIBSHDF		= 

# Code to be compiled
OBJ = \
	main.cxx

#make sure to rename the file in all the places in this make file so it will compile the desired file
all: clean TelemTest

TelemTest: $(OBJ)
		   @ echo "Building.."
		   $(CC) $(CFLAGS) -o $@ $^ $(INCLUDE) $(LIBSHDF) $(LIB)
		 		 
	
clean: 
	rm -f *.o\

.SUFFIXES:.o.c
