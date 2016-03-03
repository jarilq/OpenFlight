/*
 * ndi_interface.h
 *
 *  Created on: Oct 21, 2015
 *      Author: patipan
 */

#ifndef CONTROL_NDI_INTERFACE_H_
#define CONTROL_NDI_INTERFACE_H_

void init_control();
void get_control();
void close_control();

// Limits placed on the get_control outputs
#define		RUDDER_AUTH_MAX		0.4363		///< [rad], 25 deg, limit on get_control law output
#define		ELEVATOR_AUTH_MAX	0.4363		///< [rad], 25 deg, limit on get_control law output
#define 	AILERON_AUTH_MAX	0.4363		///< [rad], 25 deg, limit on get_control law output
#define		THROTTLE_AUTH_MAX	1.0			///< [ND],  1.0, limit on get_control law output
#define		THROTTLE_AUTH_MIN	0			///< [ND],  0, limit on get_control law output

#endif /* CONTROL_NDI_INTERFACE_H_ */
