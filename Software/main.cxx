#include <stdio.h>
#include <stdlib.h>
#include <iostream>

//#include "nav/nav_interface.h"
//#include "mission/mission_interface.h"
//#include "system_id/systemid_interface.h"
#include "aircraft/ac_config.hxx"
#include "control/ndi_interface.hxx"
#include "datalogging/datalog_interface.hxx"
#include "props.hxx"

int main(int argc, char **argv)
{
	int autop = 1;

	// Select aircraft config file
	char *FILE;
	printf("*************%s selected*************\n\n", argv[1]);
	if (strcmp(argv[1],"THOR") == 0)
	{
		FILE = (char *)"aircraft/thor.h5";
	}
	else if (strcmp(argv[1],"TYR") == 0)
	{
		FILE = (char *)"aircraft/tyr.h5";
	}
	else
	{
		printf("ERROR: Aircraft not defined\n");
		return -1;
	}

	props = new SGPropertyNode;
	double time;

	//init_dataAq();
	//init_sensorProc();
	//init_mission();
	//init_nav();
	//init_telemetry();
	init_datalog();
	get_config(FILE);
	init_control();
	//init_controlAl();
	//init_system_id(time);
	//init_actuators();

	for (int i=0; i < 3; i++)
	//while (1)
	{
		//get_dataAq();
		//get_sensorProc();
		//get_mission();
		//get_nav();
		//get_telemetry();
		get_datalog();

		if (autop == 1)
		{
			get_control();
			//get_controlAl();
			//get_system_id(time);
			//get_actuators();
		}
	}
	//close_mission();
	//close_nav();
	//close_system_id();
	close_datalog();
	close_control();
}
