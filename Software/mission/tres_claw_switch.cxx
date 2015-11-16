
#include <stdlib.h>
#include <math.h>

//#include "../globaldefs.h"
#include "../props.hxx"
#include "mission_interface.h"

//Output Properties
///********MISSION variables***********
static SGPropertyNode *claw_mode_node = NULL;
static SGPropertyNode *claw_select_node = NULL;

//Input Properties
///********INCEPTOR variables***********
static SGPropertyNode *mode_node = NULL;
static SGPropertyNode *selected_node = NULL;

//Output local variables
static double claw_mode = 0.0;
static double claw_select = 0.0;

//Input local variables
static double mode = 0.0;
static double selected = 0.0;

extern void init_mission(){
	//Variables for concatenating path
	char mission[] = "/mission";
	char inceptor[] = "/inceptor";

	//Property node initialization
	//Output variables
	claw_mode_node = fgGetNode(mission,"/claw_mode");
	claw_select_node = fgGetNode(mission,"/claw_select");

	//Input variables
	mode_node = fgGetNode(inceptor,"/mode");
	selected_node = fgGetNode(inceptor,"/select");

	// Read values from property tree
	// Input local variables
	mode = mode_node->getDoubleValue();
	selected = selected_node->getDoubleValue();

	// Initialize local output variables
	claw_mode = 0.0;
	claw_select = 0.0;
};
extern void get_mission(){};

extern void run_mission() {

	double pilot_mode = mode;
	double pilot_select = selected;

	if((pilot_mode > 0.5)&&(pilot_mode < 1.5)){
		claw_mode = 1;
	}
	else if(pilot_mode > 1.5){
		claw_mode = 2;
	}
	else{
		claw_mode = 0;
	}

	if((pilot_select > 0.5)&&(pilot_select < 1.5)){
		claw_select = 1;
	}
	else if(pilot_select > 1.5){
		claw_select = 2;
	}
	else{
		claw_select = 0;
	}

	claw_mode_node->setDoubleValue(claw_mode);
	claw_select_node->setDoubleValue(claw_select);

};

extern void close_mission() {};
