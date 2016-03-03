/*
 * misc.h
 *
 *  Created on: Oct 6, 2015
 *      Author: holly
 */

#ifndef NAV_MISC_H_
#define NAV_MISC_H_

double get_time_interval(short id);
double get_Time();
void reset_Time();
void endian_swap( unsigned char *buf, int index, int count );
double saturation(double data, double min, double max);
double polyval(double coeff[], double x, int order);
void send_status(char *status_message);
//uint16_t do_chksum(unsigned char* buffer, int start, int stop);  COMMENTED OUT BECAUSE ECLIPSE DOES NOT RECOGNIZE IT

#endif /* NAV_MISC_H_ */
