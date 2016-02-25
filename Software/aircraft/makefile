# Edit the following variables as needed
#HDF_INSTALL = /home/patipan/HDF5_DBG/hdf5
HDF_INSTALL = /home/patipan/HDF5_ARM/hdf5

EXTLIB		= -L$(HDF_INSTALL)/lib
CC          = arm-linux-gnueabihf-g++
CFLAGS      = 
LIB         = -lz -lm -lpthread -ldl

INCLUDE		= -I$(HDF_INSTALL)/include
LIBSHDF		= $(EXTLIB) $(HDF_INSTALL)/lib/libhdf5_hl.a $(HDF_INSTALL)/lib/libhdf5.a 

#make sure to rename the file in all the places in this make file so it will compile the desired file
all: clean aircraft_config
		  
aircraft_config: aircraft_config.cxx
		 		 $(CC) $(CFLAGS) -o $@ $^ $(INCLUDE) $(LIBSHDF) $(LIB)
		 		 
	
clean: 
	rm -f *.o\
	rm aircraft_config \

.SUFFIXES:.o.c
