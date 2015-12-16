
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "props.hxx"
#include "nav/nav_interface.h"
#include "mission/mission_interface.h"
//#include "control/control_interface.h"
#include "system_id/systemid_interface.h"
#include "mission/mission_interface.h"
#include "PREVIOUSglobaldefs.h"
#include "datalogging/datalog_interface.h"

using namespace std;

int main(void)
{
	int autop = 0;
	props = new SGPropertyNode;
	//struct sensordata sensorData;
	//struct control controlData;
	double time;
	/*init_dataAq();
	init_sensorProc();*/
	init_mission();
	init_nav();
	/*init_telemetry();
	init_data();*/
	//init_control(time, &sensorData, &navData, &controlData, &missionData);
	//init_controlAl();
	init_system_id(time);
	init_datalog();
	//init_actuators();

	while (1)
	{
		/*get_dataAq();
		get_sensorProc();*/
		get_mission();
		get_nav();
		/*get_telemetry();
		get_data();*/

		if (autop == 1)
		{
			//get_control(time, &sensorData, &navData, &controlData, &missionData);
			//get_controlAl();
			get_system_id(time);
			get_datalog();
			//get_actuators();
		}
	}
	close_mission();
	close_nav();
	close_system_id();
	close_datalog();
}
