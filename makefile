##############################################################################
# makefile
#
# University of Minnesota 
# Aerospace Engineering and Mechanics 
# Copyright 2011 Regents of the University of Minnesota. All rights reserved.
# 
##############################################################################

#########################################################################

CC=arm-linux-gnueabihf-g++

# Code to be compiled
OBJ =\
main.cxx

#### RULES ####
all: output display

output: $(OBJ)
	@ echo "Building..."	
	$(CC) -o $@ $^ $(LFLAGS) $(CFLAGS)
		
clean:
	-rm output

display: 
	@ echo
	@ echo "#################### UMN UAV Software ######################"
	@ echo "Successful build with the following configuration:"
	@ echo ""
	@ echo "University of Minnesota"
	@ echo "Aerospace Engineering and Mechanics"
	@ echo "Copyright 2011 Regents of the University of Minnesota. All rights reserved."
	@ echo "www.uav.aem.umn.edu" 
