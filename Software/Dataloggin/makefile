# Edit the following variables as needed
HDF_INSTALL = /usr/lib/x86_64-linux-gnu/hdf5/serial
EXTLIB = -L$(HDF_INSTALL)/lib
CC          = g++
CFLAGS      = 
LIB         = -lz -lm -lpthread -ldl

INCLUDE   = -I$(HDF_INSTALL)/include
LIBSHDF   = $(EXTLIB) $(HDF_INSTALL)/lib/libhdf5.a 

#make sure to rename the file in all the places in this make file so it will compile the desired file
all: h5_extend_write \

h5_extend_write: h5_extend_write.c
	$(CC) $(CFLAGS) -o $@ h5_extend_write.c $(INCLUDE) $(LIBSHDF) $(LIB)

clean: 
	rm -f *.h5 *.o \
        h5_extend_write \


.SUFFIXES:.o.c
