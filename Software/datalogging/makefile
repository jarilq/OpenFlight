# Edit the following variables as needed
#HDF_INSTALL = /home/patipan/HDF5_DBG/hdf5
HDF_INSTALL = /home/patipan/HDF5_ARM/hdf5

EXTLIB = -L$(HDF_INSTALL)/lib
CC          = arm-linux-gnueabihf-gcc
CFLAGS      = -std=gnu99
LIB         = -lz -lm -lpthread -ldl

INCLUDE   = -I$(HDF_INSTALL)/include
LIBSHDF   = $(EXTLIB) $(HDF_INSTALL)/lib/libhdf5_hl.a $(HDF_INSTALL)/lib/libhdf5.a 

#make sure to rename the file in all the places in this make file so it will compile the desired file
all: writeBin \
	
writeBin: writeBin.c
	$(CC) $(CFLAGS) -o $@ writeBin.c $(INCLUDE) $(LIBSHDF) $(LIB)
	
binToH5: binToH5.c
	$(CC) $(CFLAGS) -o $@ binToH5.c $(INCLUDE) $(LIBSHDF) $(LIB)
	
clean: 
	rm -f *.h5 *.o *.bin\
	rm -f writeBin \
	rm -f binToH5 \

.SUFFIXES:.o.c
