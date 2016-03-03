#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "props.hxx"
#include "nav/nav_interface.h"
#include "mission/mission_interface.h"
#include "system_id/systemid_interface.h"
#include "aircraft/ac_config.hxx"
#include "control/ndi_interface.hxx"
#include "datalogging/datalog_interface.hxx"

using namespace std;

int main(void)
{
	int autop = 1;
	props = new SGPropertyNode;
	double time;
	/*init_dataAq();
	init_sensorProc();*/
	init_mission();
	init_nav();
	//init_telemetry();
	init_datalog();
	get_config();
	init_control();
	//init_controlAl();
	init_system_id(time);
	//init_actuators();

	while (1)
	{
		/*get_dataAq();
		get_sensorProc();*/
		get_mission();
		get_nav();
		//get_telemetry();
		get_datalog();

		if (autop == 1)
		{
			get_control();
			//get_controlAl();
			get_system_id(time);
			//get_actuators();
		}
	}
	close_mission();
	close_nav();
	close_system_id();
	close_datalog();
}
