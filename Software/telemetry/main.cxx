/*
 * main.cxx
 *
 *  Created on: Apr 13, 2016
 *      Author: patipan
 */

// Code to test Mavlink

#include <mavlink.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <inttypes.h>
#include <signal.h>
#include "sys/time.h"

// TODO: read sensor variables from property nodes.

// Define Serial Port and Baudrate
#define PORT	"/dev/ttyUSB0"
#define BAUD	B57600

int initUART();
void sigalrm_handler(int sig); // Set up alarm to send command

// serial port file descripter
int File;

int heartbeat = 0;

int main()
{
	/* All Mavlink messages can be found at:
	 * mavlink.org/messages/common
	 */

	mavlink_system_t mavlink_system;

	mavlink_system.sysid = 1;
	mavlink_system.compid = MAV_COMP_ID_IMU;

	uint8_t system_type = MAV_TYPE_FIXED_WING;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	uint8_t system_mode = MAV_MODE_PREFLIGHT;
	uint32_t custom_mode = 0;
	uint8_t system_state = MAV_STATE_STANDBY;

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len;

	uint32_t time_boot; // time since system boot [ms]
	struct timeval start, current;

	printf("Opening Serial Port...\n");
	initUART();

	gettimeofday(&start, NULL);

	signal(SIGALRM, &sigalrm_handler);
	alarm(1); // call alarm every 1 second
	while(1)
	{
		if (heartbeat == 1)
		{
			gettimeofday(&current, NULL);
			time_boot = (current.tv_sec-start.tv_sec)*1000 + (current.tv_usec-start.tv_usec)/1000;
			printf("Current time: %d\n", time_boot);

			/* Send Heartbeat
			 * mavlink_msg_heartbeat_pack(SYSID, COMPID, MSGID, type, autopilot,
			 *							 base_mode, custom_mode, system_status);
			 */
			mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid,
									   &msg, system_type, autopilot_type, system_mode,
									   custom_mode, system_state);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			write(File, buf, len);
			printf("Sending heartbeat...\n");

			/* Send Attitde
			 * mavlink_msg_attitude_pack(SYSID, COMPID, MSGID, time_boot_ms, roll,
			 * 							 pitch, yaw, rollspeed, pitchspeed, yawspeed);
			 */
			mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid,
									  &msg, time_boot, 0.2, 1.0, 0.0, 0.0, 0.0, 0.0);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			write(File, buf, len);
			printf("Sending attitude...\n");

			/* Send GPS
			 * mavlink_msg_global_position_int_pack(SYSID, COMPID, MSGID, time_boot_ms, lat,
			 *					 lon, alt, relative alt, pitchspeed, yawspeed);
			 */
			mavlink_msg_global_position_int_pack(mavlink_system.sysid,
												 mavlink_system.compid,
												 &msg, time_boot, 940000000,
												 -1140000000, 250000, 0, 10,
												 2, 0, 200);
			len = mavlink_msg_to_send_buffer(buf, &msg);
			write(File, buf, len);
			printf("Sending GPS...\n");


			heartbeat = 0;
		}
	}
	return 0;
}

int initUART() {

    /* setting up UART */
    if((File = open(PORT, O_RDWR | O_NOCTTY | O_NONBLOCK))<0) {
        perror("Failed to open serial port\n");
        return -1;
    }

    struct termios options;
    tcgetattr(File, &options);
    options.c_cflag = BAUD | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(File, TCIFLUSH);
    tcsetattr(File, TCSANOW, &options);
    fcntl(File,F_SETFL,O_NONBLOCK);
    return 0;
}

void sigalrm_handler(int sig)
{
	heartbeat = 1;
	alarm(1);
}
