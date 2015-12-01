
#include <stdio.h>

#include "globaldefs.hxx"
#include "sensors/gps/gpsInterface.hxx"

int main(){
	struct gps gpsData;

	initGPS();
	while(1) {
		readGPS(&gpsData);

		if((gpsData.valid==1)&&(gpsData.newData==1)){
			printf("%.6f\t %.6f\t %.6f\n",gpsData.lat_rad*R2D,gpsData.lon_rad*R2D,gpsData.alt_m);
		}
	}




	return 0;
}
