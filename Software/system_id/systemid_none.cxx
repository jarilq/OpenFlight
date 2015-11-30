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
static SGPropertyNode *throttleCmd_ctrl_nu_node = NULL;	///< [0-1], throttle command
static SGPropertyNode *elevatorCmd_ctrl_rad_node = NULL;		///< [rad], elevator command, +TED
static SGPropertyNode *rudderCmd_ctrl_rad_node = NULL;		///< [rad], rudder command, +TEL
static SGPropertyNode *lAileronCmd_ctrl_rad_node = NULL;	///< [rad], left aileron command, +TED
static SGPropertyNode *rAileronCmd_ctrl_rad_node = NULL;	///< [rad], right aileron command, +TED
static SGPropertyNode *lFlapCmd_ctrl_rad_node = NULL;	///< [rad], left flap command, +TED
static SGPropertyNode *rFlapCmd_ctrl_rad_node = NULL;	///< [rad], right flap command, +TED

//Control Surface Variables
static double throttleCmd_ctrl_nu = 0.0;		///< [0-1], throttle command
static double elevatorCmd_ctrl_rad = 0.0;			///< [rad], elevator command, +TED
static double rudderCmd_ctrl_rad = 0.0; 		///< [rad], rudder command, +TEL
static double lAileronCmd_ctrl_rad = 0.0;		///< [rad], left aileron command, +TED
static double rAileronCmd_ctrl_rad = 0.0;		///< [rad], right aileron command, +TED
static double lFlapCmd_ctrl_rad = 0.0;		///< [rad], left flap command, +TED
static double rFlapCmd_ctrl_rad = 0.0;		///< [rad], right flap command, +TED

extern void init_system_id(double time){
	//Variables for concatenating path
	char control[] = "/control";

	//Property node initialization
	throttleCmd_ctrl_nu_node = fgGetNode(control,"/throttleCmd_ctrl_nu", 0, true);
	elevatorCmd_ctrl_rad_node = fgGetNode(control,"/elevatorCmd_ctrl_rad", 0, true);
	rudderCmd_ctrl_rad_node = fgGetNode(control,"/rudderCmd_ctrl_rad", 0, true);
	lAileronCmd_ctrl_rad_node = fgGetNode(control,"/lAileronCmd_ctrl_rad", 0, true);
	rAileronCmd_ctrl_rad_node = fgGetNode(control,"/rAileronCmd_ctrl_rad", 0, true);
	lFlapCmd_ctrl_rad_node = fgGetNode(control,"/lFlapCmd_ctrl_rad", 0, true);
	rFlapCmd_ctrl_rad_node = fgGetNode(control,"/rFlapCmd_ctrl_rad", 0, true);

	//Initialize local variables
	throttleCmd_ctrl_nu = 0.0;
	elevatorCmd_ctrl_rad = 0.0;
	rudderCmd_ctrl_rad = 0.0;
	lAileronCmd_ctrl_rad = 0.0;
	rAileronCmd_ctrl_rad = 0.0;
	lFlapCmd_ctrl_rad = 0.0;
	rFlapCmd_ctrl_rad = 0.0;

	//PUT THIS AT THE END OF CODE
	throttleCmd_ctrl_nu_node->setDoubleValue(throttleCmd_ctrl_nu);
	elevatorCmd_ctrl_rad_node->setDoubleValue(elevatorCmd_ctrl_rad);
	rudderCmd_ctrl_rad_node->setDoubleValue(rudderCmd_ctrl_rad);
	lAileronCmd_ctrl_rad_node->setDoubleValue(lAileronCmd_ctrl_rad);
	rAileronCmd_ctrl_rad_node->setDoubleValue(rAileronCmd_ctrl_rad);
	lFlapCmd_ctrl_rad_node->setDoubleValue(lFlapCmd_ctrl_rad);
	rFlapCmd_ctrl_rad_node->setDoubleValue(rFlapCmd_ctrl_rad);
};

extern void get_system_id(double time){


	//PUT THIS AT THE END OF CODE
	throttleCmd_ctrl_nu_node->setDoubleValue(throttleCmd_ctrl_nu);
	elevatorCmd_ctrl_rad_node->setDoubleValue(elevatorCmd_ctrl_rad);
	rudderCmd_ctrl_rad_node->setDoubleValue(rudderCmd_ctrl_rad);
	lAileronCmd_ctrl_rad_node->setDoubleValue(lAileronCmd_ctrl_rad);
	rAileronCmd_ctrl_rad_node->setDoubleValue(rAileronCmd_ctrl_rad);
	lFlapCmd_ctrl_rad_node->setDoubleValue(lFlapCmd_ctrl_rad);
	rFlapCmd_ctrl_rad_node->setDoubleValue(rFlapCmd_ctrl_rad);
};

extern void close_system_id() {};
