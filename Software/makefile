# Edit the following variables as needed
#HDF_INSTALL = /home/patipan/HDF5_DBG/hdf5
HDF_INSTALL = /home/patipan/HDF5_ARM/hdf5

# Code to be compiled
OBJ = \
	utils/matrix.cxx \
	utils/misc.cxx \
	utils/transfer.cxx \
	control/ndi_control.cxx \
	datalogging/datalog.cxx \
	aircraft/ac_config.cxx \
	props.cxx \
	main.cxx

EXTLIB		= -L$(HDF_INSTALL)/lib
CC          = arm-linux-gnueabihf-g++
CFLAGS      = 
LIB         = -lz -lm -lpthread -ldl -lcryptopp

INCLUDE		= -I$(HDF_INSTALL)/include
LIBSHDF		= $(EXTLIB) $(HDF_INSTALL)/lib/libhdf5_hl.a $(HDF_INSTALL)/lib/libhdf5.a 

#make sure to rename the file in all the places in this make file so it will compile the desired file
all: clean TestCode

TestCode: $(OBJ)
		  @ echo "Building.."
		  $(CC) $(CFLAGS) -o $@ $^ $(INCLUDE) $(LIBSHDF) $(LIB)
		  
binToH5: datalogging/binToH5.cxx
		 $(CC) $(CFLAGS) -o $@ $^ $(INCLUDE) $(LIBSHDF) $(LIB)
	
clean: 
	rm -f *.h5 *.o\
	rm TestCode

.SUFFIXES:.o.c
