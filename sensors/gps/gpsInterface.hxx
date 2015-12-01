
#ifndef GPSINTERFACE_HXX_
#define GPSINTERFACE_HXX_

int initGPS();

int readGPS(struct gps *gpsData_ptr);

int closeGPS();

#endif
