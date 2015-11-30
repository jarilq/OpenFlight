
#include <stdlib.h>
#include <math.h>

//#include "../globaldefs.h"
#include "../props.hxx"
#include "mission_interface.h"

//Output Properties
///********MISSION variables***********
static SGPropertyNode *clawMode_mission_nd_node = NULL;
static SGPropertyNode *clawSelect_mission_nd_node = NULL;

//Input Properties
///********INCEPTOR variables***********
static SGPropertyNode *mode_inceptor_nd_node = NULL;
static SGPropertyNode *select_inceptor_nd_node = NULL;

//Output local variables
static double clawMode_mission_nd = 0.0;
static double clawSelect_mission_nd = 0.0;

//Input local variables
static double mode_inceptor_nd = 0.0;
static double select_inceptor_nd = 0.0;

extern void init_mission(){
	//Variables for concatenating path
	char mission[] = "/mission";
	char inceptor[] = "/inceptor";

	//Property node initialization
	//Output variables
	clawMode_mission_nd_node = fgGetNode(mission,"/clawMode_mission_nd");
	clawSelect_mission_nd_node = fgGetNode(mission,"/clawSelect_mission_nd");

	//Input variables
	mode_inceptor_nd_node = fgGetNode(inceptor,"/mode_inceptor_nd");
	select_inceptor_nd_node = fgGetNode(inceptor,"/select");

	// Read values from property tree
	// Input local variables
	mode_inceptor_nd = mode_inceptor_nd_node->getDoubleValue();
	select_inceptor_nd = select_inceptor_nd_node->getDoubleValue();

	// Initialize local output variables
	clawMode_mission_nd = 0.0;
	clawSelect_mission_nd = 0.0;
};
extern void get_mission(){};

extern void run_mission() {

	double pilot_mode_inceptor_nd = mode_inceptor_nd;
	double pilot_select = select_inceptor_nd;

	if((pilot_mode_inceptor_nd > 0.5)&&(pilot_mode_inceptor_nd < 1.5)){
		clawMode_mission_nd = 1;
	}
	else if(pilot_mode_inceptor_nd > 1.5){
		clawMode_mission_nd = 2;
	}
	else{
		clawMode_mission_nd = 0;
	}

	if((pilot_select > 0.5)&&(pilot_select < 1.5)){
		clawSelect_mission_nd = 1;
	}
	else if(pilot_select > 1.5){
		clawSelect_mission_nd = 2;
	}
	else{
		clawSelect_mission_nd = 0;
	}

	clawMode_mission_nd_node->setDoubleValue(clawMode_mission_nd);
	clawSelect_mission_nd_node->setDoubleValue(clawSelect_mission_nd);

};

extern void close_mission() {};
