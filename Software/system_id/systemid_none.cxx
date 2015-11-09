/*! \file systemid_none.cxx
 *	\brief Empty System ID source code
 *
 *	\details
 *	\ingroup systemid_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: systemid_none.c 757 2012-01-04 21:57:48Z murch $
 */
#include <stdlib.h>
#include <math.h>

//#include "../globaldefs.h"
#include "systemid_interface.h"
#include "props.hxx"

//Control Surface Properties
static SGPropertyNode *dthr_node = NULL;	///< [0-1], throttle command
static SGPropertyNode *de_node = NULL;		///< [rad], elevator command, +TED
static SGPropertyNode *dr_node = NULL;		///< [rad], rudder command, +TEL
static SGPropertyNode *da_l_node = NULL;	///< [rad], left aileron command, +TED
static SGPropertyNode *da_r_node = NULL;	///< [rad], right aileron command, +TED
static SGPropertyNode *df_l_node = NULL;	///< [rad], left flap command, +TED
static SGPropertyNode *df_r_node = NULL;	///< [rad], right flap command, +TED

//Control Surface Variables
static double dthr = 0.0;		///< [0-1], throttle command
static double de = 0.0;			///< [rad], elevator command, +TED
static double dr = 0.0; 		///< [rad], rudder command, +TEL
static double da_l = 0.0;		///< [rad], left aileron command, +TED
static double da_r = 0.0;		///< [rad], right aileron command, +TED
static double df_l = 0.0;		///< [rad], left flap command, +TED
static double df_r = 0.0;		///< [rad], right flap command, +TED

extern void init_system_id(double time){
	//Property node initialization
	dthr_node = fgGetNode("/control/dthr", 0, true);
	de_node = fgGetNode("/control/de", 0, true);
	dr_node = fgGetNode("/control/dr", 0, true);
	da_l_node = fgGetNode("/control/da_l", 0, true);
	da_r_node = fgGetNode("/control/da_r", 0, true);
	df_l_node = fgGetNode("/control/df_l", 0, true);
	df_r_node = fgGetNode("/control/df_r", 0, true);

	//Initialize local variables
	dthr = 0.0;
	de = 0.0;
	dr = 0.0;
	da_l = 0.0;
	da_r = 0.0;
	df_l = 0.0;
	df_r = 0.0;

	//PUT THIS AT THE END OF CODE
	dthr_node->setDoubleValue(dthr);
	de_node->setDoubleValue(de);
	dr_node->setDoubleValue(dr);
	da_l_node->setDoubleValue(da_l);
	da_r_node->setDoubleValue(da_r);
	df_l_node->setDoubleValue(df_l);
	df_r_node->setDoubleValue(df_r);
};

extern void get_system_id(double time){


	//PUT THIS AT THE END OF CODE
	dthr_node->setDoubleValue(dthr);
	de_node->setDoubleValue(de);
	dr_node->setDoubleValue(dr);
	da_l_node->setDoubleValue(da_l);
	da_r_node->setDoubleValue(da_r);
	df_l_node->setDoubleValue(df_l);
	df_r_node->setDoubleValue(df_r);
};

