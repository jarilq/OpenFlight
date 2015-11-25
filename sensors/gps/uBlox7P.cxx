
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include "gps_interface.hxx"

#include "../../globaldefs.h"

int init_gps(struct gps *gpsData_ptr) {

	// open UART port to GPS
	if((gpsData_ptr->port = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY))<0) {
		perror("GPS UART-1: failed to open.\n");
		return -1;
	}

	// setup UART port
	struct termios options;
	tcgetattr(gpsData_ptr->port,&options);
	options.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR | ICRNL;
	tcflush(gpsData_ptr->port,TCIFLUSH);
	tcsetattr(gpsData_ptr->port,TCSANOW,&options);
}

int read_gps(struct gps *gpsData_ptr) {


}

