# Edit the following variables as needed
#HDF_INSTALL = /usr/lib/x86_64-linux-gnu/hdf5/serial
#HDF_INSTALL = /usr/lib/arm-linux-gnueabihf/hdf5/serial
#HDF_INSTALL = /home/patipan/HDF5/hdf5
HDF_INSTALL = /home/patipan/HDF5_ARM/hdf5

EXTLIB = -L$(HDF_INSTALL)/lib
#CC          = g++
CC          = arm-linux-gnueabihf-gcc
CFLAGS      = 
LIB         = -lz -lm -lpthread -ldl

INCLUDE   = -I$(HDF_INSTALL)/include
LIBSHDF   = $(EXTLIB) $(HDF_INSTALL)/lib/libhdf5_hl.a $(HDF_INSTALL)/lib/libhdf5.a 

#make sure to rename the file in all the places in this make file so it will compile the desired file
all: packetData \

extendData: extendData.c
	$(CC) $(CFLAGS) -o $@ extendData.c $(INCLUDE) $(LIBSHDF) $(LIB)
	
packetData: packetData.c
	$(CC) $(CFLAGS) -c -o packetData.o packetData.c $(INCLUDE)
	$(CC) $(CFLAGS) -c -o createData.o createData.c $(INCLUDE)
	$(CC) $(CFLAGS) -o packetData packetData.o createData.o $(INCLUDE) $(LIBSHDF) $(LIB)
	
testPacket: testPacket.c
	$(CC) $(CFLAGS) -o $@ testPacket.c $(INCLUDE) $(LIBSHDF) $(LIB)
	
writeCsv: writeCsv.c
	$(CC) $(CFLAGS) -o $@ writeCsv.c $(INCLUDE) $(LIBSHDF) $(LIB)
	
clean: 
	rm -f *.h5 *.o *.csv\
	extendData \
	packetData \
	testPacket \
	writeCsv\


.SUFFIXES:.o.c
