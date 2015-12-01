##############################################################################
# makefile
#
# University of Minnesota 
# Aerospace Engineering and Mechanics 
# Copyright 2015 Regents of the University of Minnesota. All rights reserved.
# 
##############################################################################

# compiler
CC=arm-linux-gnueabihf-g++

# configuration
GPS = uBlox7P

# code to be compiled
OBJ =\
sensors/gps/$(GPS)/$(GPS).cxx \
main.cxx

# rules
all: output display

output: $(OBJ)
	@ echo "Building..."	
	$(CC) -o $@ $^ $(LFLAGS) $(CFLAGS)
		
clean:
	-rm output

display: 
	@ echo
	@ echo "##### UMN UAV Software #####"
	@ echo "Successful build with the following configuration:"
	@ echo "GPS: "$(GPS)
	@ echo ""
	@ echo "University of Minnesota"
	@ echo "Aerospace Engineering and Mechanics"
	@ echo "Copyright 2015 Regents of the University of Minnesota. All rights reserved."
	@ echo "www.uav.aem.umn.edu"
	@ echo "" 

