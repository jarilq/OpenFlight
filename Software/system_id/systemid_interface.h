/*! \file systemid_interface.h
 *	\brief System ID interface header
 *
 *	\details This file declares the standard function prototypes for interfacing with the system ID functions.
 *	All system ID codes must include this file and implement the get_system_id() function.
 *	Auxiliary functions are declared in this header file and implemented in systemid_functions.c.
  *	\ingroup systemid_fcns
  *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: systemid_interface.h 798 2012-04-19 20:04:44Z murch $
 */

#ifndef SYSTEMID_INTERFACE_H_
#define SYSTEMID_INTERFACE_H_

/// Standard function to call the system ID function.
/*
 *	\ingroup systemid_fcns
*/
extern void init_system_id(double time 			///< [sec], time since in autopilot mode
		);

extern void get_system_id(double time 			///< [sec], time since in autopilot mode
		);

// Auxiliary Functions

/// Auxiliary function that returns a standard doublet signal
/*!
 *	\sa doublet121()
 *	\ingroup systemid_fcns
*/
double doublet(double t0_sec,		///< [sec], start time
		double currentTime_sec,		///< [sec], current time
		double duration,		///< [sec], period of doublet pulse
		double amplitude_rad		///< [rad], magnitude of doublet pulse
		);

/// Auxiliary function that returns a 1-2-1 doublet signal
/*!
 *	\sa doublet()
 *	\ingroup systemid_fcns
*/
double doublet121(double t0_sec,	///< [sec], start time
		double currentTime_sec,		///< [sec], current time
		double durPulse1_sec,		///< [sec], duration of first pulse
		double durPulse2_sec,		///< [sec], duration of second pulse
		double durPulse3_sec,		///< [sec], duration of third pulse
		double amplitude_rad		///< [rad], magnitude of pulse
		);

/// Auxiliary function that adds a orthogonal multi-sine to one surface
/*!
 *	\sa two_multi_sine(), three_multi_sine()
 *	\ingroup systemid_fcns
*/
void one_multi_sine (double t_sec,	///< [sec], current time
		double *dsurf,			///< pointer to control surface
		double amp_rad			///< [rad], amplitude of sine signal
		);

/// Auxiliary function that adds an orthogonal multi-sine to two surfaces
/*!
 *	\sa one_multi_sine(), three_multi_sine()
 *	\ingroup systemid_fcns
*/
void two_multi_sine (double t,	///< [sec], current time
		double *dsurf1,			///< pointer to control surface 1
		double *dsurf2,			///< pointer to control surface 2
		double amp1_rad,			///< [rad], amplitude of sine signal 1
		double amp2_rad				///< [rad], amplitude of sine signal 2
		);

/// Auxiliary function that adds an orthogonal multi-sine to three surfaces
/*!
 *	\sa two_multi_sine(), three_multi_sine()
 *	\ingroup systemid_fcns
*/
void three_multi_sine (double t_sec,	///< [sec], current time
		double *elevatorSurf,					///< pointer to elevator control surface
		double *aileronSurf,					///< pointer to aileron control surface
		double *rudderSurf					///< pointer to rudder control surface
		);

extern void close_system_id();

#endif /* SYSTEMID_INTERFACE_H_ */
