
#include <unistd.h>

#include "globaldefs.h"
#include "sensors/gps/gps_interface.hxx"

int main(){
	struct gps gpsData;

	init_gps(&gpsData);
	while(1) {
		read_gps(&gpsData);
	}




	return 0;
}
