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
#include "../props.hxx"

//Control Surface Properties
static SGPropertyNode *throcmd_control_nu_node = NULL;	///< [0-1], throttle command
static SGPropertyNode *elevcmd_control_rads_node = NULL;		///< [rad], elevator command, +TED
static SGPropertyNode *rudcmd_control_rads_node = NULL;		///< [rad], rudder command, +TEL
static SGPropertyNode *lailcmd_control_rads_node = NULL;	///< [rad], left aileron command, +TED
static SGPropertyNode *railcmd_control_rads_node = NULL;	///< [rad], right aileron command, +TED
static SGPropertyNode *lfacmd_control_rads_node = NULL;	///< [rad], left flap command, +TED
static SGPropertyNode *rflacmd_control_rads_node = NULL;	///< [rad], right flap command, +TED

//Control Surface Variables
static double throcmd_control_nu = 0.0;		///< [0-1], throttle command
static double elevcmd_control_rads = 0.0;			///< [rad], elevator command, +TED
static double rudcmd_control_rads = 0.0; 		///< [rad], rudder command, +TEL
static double lailcmd_control_rads = 0.0;		///< [rad], left aileron command, +TED
static double railcmd_control_rads = 0.0;		///< [rad], right aileron command, +TED
static double lfacmd_control_rads = 0.0;		///< [rad], left flap command, +TED
static double rflacmd_control_rads = 0.0;		///< [rad], right flap command, +TED

extern void init_system_id(double time){
	//Property node initialization
	throcmd_control_nu_node = fgGetNode("/control/throcmd_control_nu", 0, true);
	elevcmd_control_rads_node = fgGetNode("/control/de", 0, true);
	rudcmd_control_rads_node = fgGetNode("/control/rudcmd_control_rads", 0, true);
	lailcmd_control_rads_node = fgGetNode("/control/lailcmd_control_rads", 0, true);
	railcmd_control_rads_node = fgGetNode("/control/railcmd_control_rads", 0, true);
	lfacmd_control_rads_node = fgGetNode("/control/lfacmd_control_rads", 0, true);
	rflacmd_control_rads_node = fgGetNode("/control/rflacmd_control_rads", 0, true);

	//Initialize local variables
	throcmd_control_nu = 0.0;
	elevcmd_control_rads = 0.0;
	rudcmd_control_rads = 0.0;
	lailcmd_control_rads = 0.0;
	railcmd_control_rads = 0.0;
	lfacmd_control_rads = 0.0;
	rflacmd_control_rads = 0.0;

	//PUT THIS AT THE END OF CODE
	throcmd_control_nu_node->setDoubleValue(throcmd_control_nu);
	elevcmd_control_rads_node->setDoubleValue(elevcmd_control_rads);
	rudcmd_control_rads_node->setDoubleValue(rudcmd_control_rads);
	lailcmd_control_rads_node->setDoubleValue(lailcmd_control_rads);
	railcmd_control_rads_node->setDoubleValue(railcmd_control_rads);
	lfacmd_control_rads_node->setDoubleValue(lfacmd_control_rads);
	rflacmd_control_rads_node->setDoubleValue(rflacmd_control_rads);
};

extern void get_system_id(double time){


	//PUT THIS AT THE END OF CODE
	throcmd_control_nu_node->setDoubleValue(throcmd_control_nu);
	elevcmd_control_rads_node->setDoubleValue(elevcmd_control_rads);
	rudcmd_control_rads_node->setDoubleValue(rudcmd_control_rads);
	lailcmd_control_rads_node->setDoubleValue(lailcmd_control_rads);
	railcmd_control_rads_node->setDoubleValue(railcmd_control_rads);
	lfacmd_control_rads_node->setDoubleValue(lfacmd_control_rads);
	rflacmd_control_rads_node->setDoubleValue(rflacmd_control_rads);
};

