/*! \file EKF_15state.cxx
 *	\brief 15 state EKF navigation filter
 *
 *	\details  15 state EKF navigation filter using loosely integrated INS/GPS architecture.
 * 	Time update is done after every IMU data acquisition and GPS measurement
 * 	update is done every time the new data flag in the GPS data packet is set. Designed by Adhika Lie.
 *	Attitude is parameterized using quaternions.
 *	Estimates IMU bias errors.
 *	\ingroup nav_fcns
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: EKF_15state_quat.c 911 2012-10-08 15:00:59Z lie $
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
//#include <cyg/posix/pthread.h>
//#include <cyg/kernel/kapi.h>
//#include <cyg/cpuload/cpuload.h>

//#include "../globaldefs.h"
#include "../props.hxx"
//#include "../extern_vars.h"
#include "../utils/matrix.h"
#include "../utils/misc.h"

#include "nav_functions.hxx"
#include "nav_interface.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//error characteristics of navigation parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define		SIG_W_AX	1		//1 m/s^2
#define		SIG_W_AY	1
#define		SIG_W_AZ	1
#define		SIG_W_GX	0.00524		//0.3 deg/s
#define		SIG_W_GY	0.00524
#define		SIG_W_GZ	0.00524
#define		SIG_A_D		0.1	    	//5e-2*g
#define		TAU_A		100
#define		SIG_G_D		0.00873		//0.1 deg/s
#define		TAU_G		50

#define		SIG_GPS_P_NE 3
#define		SIG_GPS_P_D  5
#define		SIG_GPS_V	 0.5

#define		P_P_INIT	10.0
#define		P_V_INIT	1.0
#define		P_A_INIT	0.34906		// 20 deg
#define		P_HDG_INIT	3.14159		//180 deg
#define		P_AB_INIT	0.9810		//0.5*g
#define		P_GB_INIT	0.01745		//5 deg/s

#define     Rew     		6.359058719353925e+006      //earth radius
#define     Rns     		6.386034030458164e+006      //earth radius

/*+++++++++++++++++++++++++++++++++++++++++++++
 * added constant values from global defs
 ++++++++++++++++++++++++++++++++++++++++++++++*/
#define NSECS_PER_SEC	1000000000 		///< [nsec/sec] nanoseconds per second */
const double D2R = 0.017453292519943;	///< [rad] degrees to radians */
#define R2D			57.295779513082323	///< [deg] radians to degrees */
#define PSI_TO_KPA  6.89475729  		///< [KPa] PSI to KPa */
#define	g			9.814				///< [m/sec^2] gravity */
#define g2      	19.62   			///< [m/sec^2] 2*g */
#define PI      	3.14159265358979    ///< pi */
#define PI2     	6.28318530717958	///< pi*2 */
#define half_pi		1.57079632679490	///< pi/2 */
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef FALSE
	#define FALSE 0
#endif

static MATRIX C_N2B, C_B2N;
static MATRIX F, PHI, P, G, K, H;
static MATRIX Rw, Q, Qw, R;
static MATRIX x, y, eul, Rbodtonav, grav, f_b, om_ib, nr;
static MATRIX pos_ref, pos_ins_ecef, pos_ins_ned;
static MATRIX pos_gps, pos_gps_ecef, pos_gps_ned;
static MATRIX I15, I3, ImKH, KRKt;
static MATRIX dx, a_temp31, b_temp31, temp33, atemp33, temp615, temp1515, temp66, atemp66, temp156, temp1512;

static double quat[4];

static double denom, Re, Rn;
static double tprev;

// Input Properties
//************IMU variables****************
static SGPropertyNode *p_imu_rps_node = NULL; 					///< [rad/sec], body X axis angular rate (roll)
static SGPropertyNode *q_imu_rps_node = NULL; 					///< [rad/sec], body Y axis angular rate (pitch)
static SGPropertyNode *r_imu_rps_node = NULL; 					///< [rad/sec], body Z axis angular rate (yaw)
static SGPropertyNode *xAcceleration_imu_mpsSq_node = NULL; 	///< [m/sec^2], body X axis acceleration
static SGPropertyNode *yAcceleration_imu_mpsSq_node = NULL; 	///< [m/sec^2], body Y axis acceleration
static SGPropertyNode *zAcceleration_imu_mpsSq_node = NULL; 	///< [m/sec^2], body Z axis acceleration
static SGPropertyNode *time_imu_sec_node = NULL;				///< [sec], timestamp of IMU data

//************GPS variables****************
static SGPropertyNode *latitude_gps_degs_node = NULL; 			///< [deg], Geodetic latitude
static SGPropertyNode *longitude_gps_degs_node = NULL; 			///< [deg], Geodetic longitude
static SGPropertyNode *altitude_gps_m_node = NULL; 				///< [m], altitude relative to WGS84
static SGPropertyNode *northVelocity_gps_mps_node = NULL; 		///< [m/sec], North velocity
static SGPropertyNode *eastVelocity_gps_mps_node = NULL; 		///< [m/sec], East velocity
static SGPropertyNode *downVelocity_gps_mps_node = NULL; 		///< [m/sec], Down velocity
static SGPropertyNode *newData_node = NULL;						///< [bool], flag set when GPS data has been updated

// Output Properties
//************NAV variables****************
static SGPropertyNode *latitude_nav_rads_node = NULL; 			///< [rad], geodetic latitude estimate
static SGPropertyNode *longitude_nav_rads_node = NULL;			///< [rad], geodetic longitude estimate
static SGPropertyNode *altitude_nav_m_node = NULL; 				///< [m], altitude relative to WGS84 estimate
static SGPropertyNode *northVelocity_nav_mps_node = NULL;		///< [m/sec], north velocity estimate
static SGPropertyNode *eastVelocity_nav_mps_node = NULL; 		///< [m/sec], east velocity estimate
static SGPropertyNode *downVelocity_nav_mps_node = NULL; 		///< [m/sec], down velocity estimate
static SGPropertyNode *rollAngle_nav_rads_node = NULL; 			///< [rad], Euler roll angle estimate
static SGPropertyNode *pitchAngle_nav_rads_node = NULL; 		///< [rad], Euler pitch angle estimate
static SGPropertyNode *yawAngle_nav_rads_node = NULL; 			///< [rad], Euler yaw angle estimate
static SGPropertyNode *quat_nav_node[4] = {NULL, NULL, NULL, NULL};// = NULL; //Do arrays need NULL?		///< Quaternions estimate
static SGPropertyNode *accelerometerBias_nav_mpsSq_node[3] = {NULL, NULL, NULL};			///< [m/sec^2], accelerometer bias estimate
static SGPropertyNode *gyroBias_nav_rps_node[3] = {NULL, NULL, NULL};					///< [rad/sec], rate gyro bias estimate
static SGPropertyNode *covariancePosition_nav_rads_node[3] = {NULL, NULL, NULL};			///< [rad], covariance estimate for position
static SGPropertyNode *covarianceVelocity_nav_rads_node[3] = {NULL, NULL, NULL};			///< [rad], covariance estimate for velocity
static SGPropertyNode *covarianceAngles_nav_rads_node[3] = {NULL, NULL, NULL};			///< [rad], covariance estimate for angles
static SGPropertyNode *covarianceAccelBias_nav_rads_node[3] = {NULL, NULL, NULL};		///< [rad], covariance estimate for accelerometer bias
static SGPropertyNode *covarianceGyroBias_nav_rads_node[3] = { NULL, NULL, NULL};			///< [rad], covariance estimate for rate gyro bias
static SGPropertyNode *err_type_nav_node = NULL; //fgGetNode("/nav/err_type", 0, true); 						///< NAV filter status

// Input local variables
//************IMU variables****************
static double p_imu_rps = 0.0; 		///< [rad/sec], body X axis angular rate (roll)
static double q_imu_rps = 0.0; 		///< [rad/sec], body Y axis angular rate (pitch)
static double r_imu_rps = 0.0; 							///< [rad/sec], body Z axis angular rate (yaw)
static double xAcceleration_imu_mpsSq = 0.0; 	///< [m/sec^2], body X axis acceleration
static double yAcceleration_imu_mpsSq = 0.0; 	///< [m/sec^2], body Y axis acceleration
static double zAcceleration_imu_mpsSq = 0.0; 	///< [m/sec^2], body Z axis acceleration
static double time_imu_sec = 0.0; 				///< [sec], timestamp of IMU data

//************GPS variables****************
static double latitude_gps_degs = 0.0; 			///< [deg], Geodetic latitude
static double longitude_gps_degs = 0.0;			///< [deg], Geodetic longitude
static double altitude_gps_m = 0.0; 			///< [m], altitude relative to WGS84
static double northVelocity_gps_mps = 0.0; 		///< [m/sec], North velocity
static double eastVelocity_gps_mps = 0.0; 		///< [m/sec], East velocity
static double downVelocity_gps_mps = 0.0;		///< [m/sec], Down velocity
static unsigned short newData = 0.0;			///< [bool], flag set when GPS data has been updated

// Output local variables
//************NAV variables****************
static double latitude_nav_rads = 0.0; 			///< [rad], geodetic latitude estimate
static double longitude_nav_rads = 0.0;			///< [rad], geodetic longitude estimate
static double altitude_nav_m = 0.0;				///< [m], altitude relative to WGS84 estimate
static double northVelocity_nav_mps = 0.0; 		///< [m/sec], north velocity estimate
static double eastVelocity_nav_mps = 0.0; 		///< [m/sec], east velocity estimate
static double downVelocity_nav_mps = 0.0; 		///< [m/sec], down velocity estimate
static double rollAngle_nav_rads = 0.0; 		///< [rad], Euler roll angle estimate
static double pitchAngle_nav_rads = 0.0; 		///< [rad], Euler pitch angle estimate
static double yawAngle_nav_rads = 0.0; 			///< [rad], Euler yaw angle estimate
static double quat_nav[4] = {0.0, 0.0, 0.0, 0.0};		///< Quaternions estimate
static double accelerometerBias_nav_mpsSq[3] = {0.0, 0.0, 0.0};			///< [m/sec^2], accelerometer bias estimate
static double gyroBias_nav_rps[3] = {0.0, 0.0, 0.0};			///< [rad/sec], rate gyro bias estimate
static double covariancePosition_nav_rads[3] = {0.0, 0.0, 0.0};				///< [rad], covariance estimate for position
static double covarianceVelocity_nav_rads[3] = {0.0, 0.0, 0.0};				///< [rad], covariance estimate for velocity
static double covarianceAngles_nav_rads[3] = {0.0, 0.0, 0.0};			///< [rad], covariance estimate for angles
static double covarianceAccelBias_nav_rads[3] = {0.0, 0.0, 0.0};				///< [rad], covariance estimate for accelerometer bias
static double covarianceGyroBias_nav_rads[3] = {0.0, 0.0, 0.0};			///< [rad], covariance estimate for rate gyro bias
enum   errdefs	{
	data_valid,			///< Data valid
	gps_aided,			///< NAV filter, GPS aided
	TU_only
	};
static errdefs err_type_nav = data_valid;

void init_nav(){
	/*++++++++++++++++++++++++++++++++++++++++++++++++
	 *matrix creation for navigation computation
	 *++++++++++++++++++++++++++++++++++++++++++++++++*/
	F 			= mat_creat(15,15,ZERO_MATRIX);		// State Matrix
	PHI 		= mat_creat(15,15,ZERO_MATRIX);		// State transition Matrix

	P 			= mat_creat(15,15,ZERO_MATRIX);		// Covariance Matrix
	G			= mat_creat(15,12,ZERO_MATRIX);		// for Process Noise Transformation
	Rw 			= mat_creat(12,12,ZERO_MATRIX);
	Qw 			= mat_creat(15,15,ZERO_MATRIX);
	Q 			= mat_creat(15,15,ZERO_MATRIX);		// Process Noise Matrix
	R 			= mat_creat(6,6,ZERO_MATRIX);		// GPS Measurement Noise matrix

	x			= mat_creat(15,1,ZERO_MATRIX);
	y			= mat_creat(6,1,ZERO_MATRIX);		// GPS Measurement
	eul			= mat_creat(3,1,ZERO_MATRIX);
	Rbodtonav 	= mat_creat(3,3,ZERO_MATRIX);
	grav		= mat_creat(3,1,ZERO_MATRIX);		// Gravity Model
	f_b 		= mat_creat(3,1,ZERO_MATRIX);
	om_ib		= mat_creat(3,1,ZERO_MATRIX);
	nr			= mat_creat(3,1,ZERO_MATRIX);		// Nav Transport Rate

	K 			= mat_creat(15,6,ZERO_MATRIX);		// Kalman Gain
	H			= mat_creat(6,15,ZERO_MATRIX);

	C_N2B 		= mat_creat(3,3,ZERO_MATRIX);		// DCM
	C_B2N 		= mat_creat(3,3,ZERO_MATRIX);		// DCM Transpose

	pos_ref		= mat_creat(3,1,ZERO_MATRIX);
	pos_ins_ecef= mat_creat(3,1,ZERO_MATRIX);
	pos_ins_ned	= mat_creat(3,1,ZERO_MATRIX);

	pos_gps		= mat_creat(3,1,ZERO_MATRIX);
	pos_gps_ecef= mat_creat(3,1,ZERO_MATRIX);
	pos_gps_ned	= mat_creat(3,1,ZERO_MATRIX);

	I15			= mat_creat(15,15,UNIT_MATRIX);		// Identity
	I3			= mat_creat(3,3,UNIT_MATRIX);		// Identity
	ImKH		= mat_creat(15,15,ZERO_MATRIX);
	KRKt		= mat_creat(15,15,ZERO_MATRIX);

	dx 			= mat_creat(3,1,ZERO_MATRIX);		// Temporary to get dxdt
	a_temp31	= mat_creat(3,1,ZERO_MATRIX);		// Temporary
	b_temp31	= mat_creat(3,1,ZERO_MATRIX);		// Temporary
	temp33		= mat_creat(3,3,ZERO_MATRIX);		// Temporary
	atemp33		= mat_creat(3,3,ZERO_MATRIX);		// Temporary
	temp1515	= mat_creat(15,15,ZERO_MATRIX);
	temp615 	= mat_creat(6,15,ZERO_MATRIX);
	temp66		= mat_creat(6,6,ZERO_MATRIX);
	atemp66 	= mat_creat(6,6,ZERO_MATRIX);
	temp156		= mat_creat(15,6,ZERO_MATRIX);
	temp1512	= mat_creat(15,12,ZERO_MATRIX);

	// Property node initialization
	//************IMU variables****************
	p_imu_rps_node = fgGetNode("/imu/p_imu_rps", 0, true);								///< [rad/sec], body X axis angular rate (roll)
	q_imu_rps_node = fgGetNode("/imu/q_imu_rps", 0, true);								///< [rad/sec], body Y axis angular rate (pitch)
	r_imu_rps_node = fgGetNode("/imu/r_imu_rps", 0, true);								///< [rad/sec], body Z axis angular rate (yaw)
	xAcceleration_imu_mpsSq_node = fgGetNode("/imu/xAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body X axis acceleration
	yAcceleration_imu_mpsSq_node = fgGetNode("/imu/yAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body Y axis acceleration
	zAcceleration_imu_mpsSq_node = fgGetNode("/imu/zAcceleration_imu_mpsSq", 0, true);	///< [m/sec^2], body Z axis acceleration
	time_imu_sec_node = fgGetNode("/imu/time_imu_sec", 0, true);							///< [sec], timestamp of IMU data

	//************GPS variables****************
	latitude_gps_degs_node = fgGetNode("/gps/latitude_gps_degs", 0, true);				///< [deg], Geodetic latitude
	longitude_gps_degs_node = fgGetNode("/gps/longitude_gps_degs", 0, true);				///< [deg], Geodetic longitude
	altitude_gps_m_node = fgGetNode("/gps/altitude_gps_m", 0, true);						///< [m], altitude relative to WGS84
	northVelocity_gps_mps_node = fgGetNode("/gps/northVelocity_gps_mps", 0, true);		///< [m/sec], North velocity
	eastVelocity_gps_mps_node = fgGetNode("/gps/eastVelocity_gps_mps", 0, true);			///< [m/sec], East velocity
	downVelocity_gps_mps_node = fgGetNode("/gps/downVelocity_gps_mps", 0, true);			///< [m/sec], Down velocity
	newData_node = fgGetNode("/gps/newData", true);		///< [bool], flag set when GPS data has been updated

	//************NAV variables****************
	latitude_nav_rads_node = fgGetNode("/nav/latitude_nav_rads", true);				///< [rad], geodetic latitude estimate
	longitude_nav_rads_node = fgGetNode("/nav/longitude_nav_rads", true);			///< [rad], geodetic longitude estimate
	altitude_nav_m_node = fgGetNode("/nav/altitude_nav_m", true);					///< [m], altitude relative to WGS84 estimate
	northVelocity_nav_mps_node = fgGetNode("/nav/northVelocity_nav_mps", true);		///< [m/sec], north velocity estimate
	eastVelocity_nav_mps_node = fgGetNode("/nav/eastVelocity_nav_mps", true);		///< [m/sec], east velocity estimate
	downVelocity_nav_mps_node = fgGetNode("/nav/downVelocity_nav_mps", true);		///< [m/sec], down velocity estimate
	rollAngle_nav_rads_node = fgGetNode("/nav/rollAngle_nav_rads", true);			///< [rad], Euler roll angle estimate
	pitchAngle_nav_rads_node = fgGetNode("/nav/pitchAngle_nav_rads", true);			///< [rad], Euler pitch angle estimate
	yawAngle_nav_rads_node = fgGetNode("/nav/yawAngle_nav_rads", true);				///< [rad], Euler yaw angle estimate
	///< Quaternions estimate
	quat_nav_node[0] = fgGetNode("/nav/quat_nav", 0, true);
	quat_nav_node[1] = fgGetNode("/nav/quat_nav", 1, true);
	quat_nav_node[2] = fgGetNode("/nav/quat_nav", 2, true);
	quat_nav_node[3] = fgGetNode("/nav/quat_nav", 3, true);
	///< [m/sec^2], accelerometer bias estimate
	accelerometerBias_nav_mpsSq_node[0] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 0, true);
	accelerometerBias_nav_mpsSq_node[1] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 1, true);
	accelerometerBias_nav_mpsSq_node[2] = fgGetNode("/nav/accelerometerBias_nav_mpsSq", 2, true);
	///< [rad/sec], rate gyro bias estimate
	gyroBias_nav_rps_node[0] = fgGetNode("/nav/gyroBias_nav_rps", 0, true);
	gyroBias_nav_rps_node[1] = fgGetNode("/nav/gyroBias_nav_rps", 1, true);
	gyroBias_nav_rps_node[2] = fgGetNode("/nav/gyroBias_nav_rps", 2, true);
	///< [rad], covariance estimate for position
	covariancePosition_nav_rads_node[0] = fgGetNode("/nav/covariancePosition_nav_rads", 0, true);
	covariancePosition_nav_rads_node[1] = fgGetNode("/nav/covariancePosition_nav_rads", 1, true);
	covariancePosition_nav_rads_node[2] = fgGetNode("/nav/covariancePosition_nav_rads", 2, true);
	///< [rad], covariance estimate for velocity
	covarianceVelocity_nav_rads_node[0] = fgGetNode("/nav/covarianceVelocity_nav_rads", 0, true);
	covarianceVelocity_nav_rads_node[1] = fgGetNode("/nav/covarianceVelocity_nav_rads", 1, true);
	covarianceVelocity_nav_rads_node[2] = fgGetNode("/nav/covarianceVelocity_nav_rads", 2, true);
	///< [rad], covariance estimate for angles
	covarianceAngles_nav_rads_node[0] = fgGetNode("/nav/covarianceAngles_nav_rads", 0, true);
	covarianceAngles_nav_rads_node[1] = fgGetNode("/nav/covarianceAngles_nav_rads", 1, true);
	covarianceAngles_nav_rads_node[2] = fgGetNode("/nav/covarianceAngles_nav_rads", 2, true);
	///< [rad], covariance estimate for accelerometer bias
	covarianceAccelBias_nav_rads_node[0] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 0, true);
	covarianceAccelBias_nav_rads_node[1] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 1, true);
	covarianceAccelBias_nav_rads_node[2] = fgGetNode("/nav/covarianceAccelBias_nav_rads", 2, true);
	///< [rad], covariance estimate for rate gyro bias
	covarianceGyroBias_nav_rads_node[0] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 0, true);
	covarianceGyroBias_nav_rads_node[1] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 1, true);
	covarianceGyroBias_nav_rads_node[2] = fgGetNode("/nav/covarianceGyroBias_nav_rads", 2, true);
	err_type_nav_node = fgGetNode("/nav/err_type", 0, true);


	// Read values from property tree
	// Input local variables
	//************IMU variables****************
	p_imu_rps = p_imu_rps_node->getDoubleValue(); 		///< [rad/sec], body X axis angular rate (roll)
	q_imu_rps = q_imu_rps_node->getDoubleValue(); 		///< [rad/sec], body Y axis angular rate (pitch)
	r_imu_rps = r_imu_rps_node->getDoubleValue(); 							///< [rad/sec], body Z axis angular rate (yaw)
	xAcceleration_imu_mpsSq = xAcceleration_imu_mpsSq_node->getDoubleValue(); 	///< [m/sec^2], body X axis acceleration
	yAcceleration_imu_mpsSq = yAcceleration_imu_mpsSq_node->getDoubleValue(); 	///< [m/sec^2], body Y axis acceleration
	zAcceleration_imu_mpsSq = zAcceleration_imu_mpsSq_node->getDoubleValue(); 	///< [m/sec^2], body Z axis acceleration
	time_imu_sec = time_imu_sec_node->getDoubleValue(); 				///< [sec], timestamp of IMU data

	//************GPS variables****************
	latitude_gps_degs = latitude_gps_degs_node->getDoubleValue(); //fgGetNode("/gps/latitude_gps_degs", 0, true);				///< [deg], Geodetic latitude
	longitude_gps_degs = longitude_gps_degs_node->getDoubleValue();//fgGetNode("/gps/longitude_gps_degs", 0, true);				///< [deg], Geodetic longitude
	altitude_gps_m = altitude_gps_m_node->getDoubleValue(); //fgGetNode("/gps/altitude_gps_m", 0, true);						///< [m], altitude relative to WGS84
	northVelocity_gps_mps = northVelocity_gps_mps_node->getDoubleValue(); //fgGetNode("/gps/northVelocity_gps_mps", 0, true);		///< [m/sec], North velocity
	eastVelocity_gps_mps = eastVelocity_gps_mps_node->getDoubleValue(); //fgGetNode("/gps/eastVelocity_gps_mps", 0, true);			///< [m/sec], East velocity
	downVelocity_gps_mps = downVelocity_gps_mps_node->getDoubleValue(); //fgGetNode("/gps/downVelocity_gps_mps", 0, true);			///< [m/sec], Down velocity
	newData = newData_node->getDoubleValue();		///< [bool], flag set when GPS data has been updated

	// Initialize local output variables
	//************NAV variables****************
	latitude_nav_rads = 0.0; 			///< [rad], geodetic latitude estimate
	longitude_nav_rads = 0.0;			///< [rad], geodetic longitude estimate
	altitude_nav_m = 0.0;				///< [m], altitude relative to WGS84 estimate
	northVelocity_nav_mps = 0.0; 		///< [m/sec], north velocity estimate
	eastVelocity_nav_mps = 0.0; 		///< [m/sec], east velocity estimate
	downVelocity_nav_mps = 0.0; 		///< [m/sec], down velocity estimate
	rollAngle_nav_rads = 0.0; 		///< [rad], Euler roll angle estimate
	pitchAngle_nav_rads = 0.0; 		///< [rad], Euler pitch angle estimate
	yawAngle_nav_rads = 0.0; 			///< [rad], Euler yaw angle estimate
	///< Quaternions estimate
	quat_nav[0] = 0.0;
	quat_nav[1] = 0.0;
	quat_nav[2] = 0.0;
	quat_nav[3] = 0.0;
	///< [m/sec^2], accelerometer bias estimate
	accelerometerBias_nav_mpsSq[0] = 0.0;
	accelerometerBias_nav_mpsSq[1] = 0.0;
	accelerometerBias_nav_mpsSq[2] = 0.0;
	///< [rad/sec], rate gyro bias estimate
	gyroBias_nav_rps[0] = 0.0;
	gyroBias_nav_rps[1] = 0.0;
	gyroBias_nav_rps[2] = 0.0;;
	///< [rad], covariance estimate for position
	covariancePosition_nav_rads[0] = 0.0;
	covariancePosition_nav_rads[1] = 0.0;
	covariancePosition_nav_rads[2] = 0.0;
	///< [rad], covariance estimate for velocity
	covarianceVelocity_nav_rads[0] = 0.0;
	covarianceVelocity_nav_rads[1] = 0.0;
	covarianceVelocity_nav_rads[2] = 0.0;
	///< [rad], covariance estimate for angles
	covarianceAngles_nav_rads[0] = 0.0;
	covarianceAngles_nav_rads[1] = 0.0;
	covarianceAngles_nav_rads[2] = 0.0;
	///< [rad], covariance estimate for accelerometer bias
	covarianceAccelBias_nav_rads[0] = 0.0;
	covarianceAccelBias_nav_rads[1] = 0.0;
	covarianceAccelBias_nav_rads[2] = 0.0;
	///< [rad], covariance estimate for rate gyro bias
	covarianceGyroBias_nav_rads[0] = 0.0;
	covarianceGyroBias_nav_rads[1] = 0.0;
	covarianceGyroBias_nav_rads[2] = 0.0;

	// Assemble the matrices
	// .... gravity, g
	grav[2][0] = g;

	// ... H
	H[0][0] = 1.0; 	H[1][1] = 1.0; 	H[2][2] = 1.0;
	H[3][3] = 1.0; 	H[4][4] = 1.0; 	H[5][5] = 1.0;

	// ... Rw
	Rw[0][0] = SIG_W_AX*SIG_W_AX;		Rw[1][1] = SIG_W_AY*SIG_W_AY;			Rw[2][2] = SIG_W_AZ*SIG_W_AZ;
	Rw[3][3] = SIG_W_GX*SIG_W_GX;		Rw[4][4] = SIG_W_GY*SIG_W_GY;			Rw[5][5] = SIG_W_GZ*SIG_W_GZ;
	Rw[6][6] = 2*SIG_A_D*SIG_A_D/TAU_A;	Rw[7][7] = 2*SIG_A_D*SIG_A_D/TAU_A;		Rw[8][8] = 2*SIG_A_D*SIG_A_D/TAU_A;
	Rw[9][9] = 2*SIG_G_D*SIG_G_D/TAU_G;	Rw[10][10] = 2*SIG_G_D*SIG_G_D/TAU_G;	Rw[11][11] = 2*SIG_G_D*SIG_G_D/TAU_G;


	// ... P (initial)
	P[0][0] = P_P_INIT*P_P_INIT; 		P[1][1] = P_P_INIT*P_P_INIT; 		P[2][2] = P_P_INIT*P_P_INIT;
	P[3][3] = P_V_INIT*P_V_INIT; 		P[4][4] = P_V_INIT*P_V_INIT; 		P[5][5] = P_V_INIT*P_V_INIT;
	P[6][6] = P_A_INIT*P_A_INIT; 		P[7][7] = P_A_INIT*P_A_INIT; 		P[8][8] = P_HDG_INIT*P_HDG_INIT;
	P[9][9] = P_AB_INIT*P_AB_INIT; 		P[10][10] = P_AB_INIT*P_AB_INIT; 	P[11][11] = P_AB_INIT*P_AB_INIT;
	P[12][12] = P_GB_INIT*P_GB_INIT; 	P[13][13] = P_GB_INIT*P_GB_INIT; 	P[14][14] = P_GB_INIT*P_GB_INIT;

	// ... update P in get_nav
	covariancePosition_nav_rads[0] = P[0][0];	covariancePosition_nav_rads[1] = P[1][1];	covariancePosition_nav_rads[2] = P[2][2];
	covarianceVelocity_nav_rads[0] = P[3][3];	covarianceVelocity_nav_rads[1] = P[4][4];	covarianceVelocity_nav_rads[2] = P[5][5];
	covarianceAngles_nav_rads[0] = P[6][6];		covarianceAngles_nav_rads[1] = P[7][7];		covarianceAngles_nav_rads[2] = P[8][8];

	covarianceAccelBias_nav_rads[0] = P[9][9];	covarianceAccelBias_nav_rads[1] = P[10][10]; 	covarianceAccelBias_nav_rads[2] = P[11][11];
	covarianceGyroBias_nav_rads[0] = P[12][12];	covarianceGyroBias_nav_rads[1] = P[13][13]; 	covarianceGyroBias_nav_rads[2] = P[14][14];

	// ... R
	R[0][0] = SIG_GPS_P_NE*SIG_GPS_P_NE;	R[1][1] = SIG_GPS_P_NE*SIG_GPS_P_NE;	R[2][2] = SIG_GPS_P_D*SIG_GPS_P_D;
	R[3][3] = SIG_GPS_V*SIG_GPS_V;			R[4][4] = SIG_GPS_V*SIG_GPS_V;			R[5][5] = SIG_GPS_V*SIG_GPS_V;


	// .. then initialize states with GPS Data
	latitude_nav_rads = latitude_gps_degs*D2R;
	longitude_nav_rads = longitude_gps_degs*D2R;
	altitude_nav_m = altitude_gps_m;

	northVelocity_nav_mps = northVelocity_gps_mps;
	eastVelocity_nav_mps = eastVelocity_gps_mps;
	downVelocity_nav_mps = downVelocity_gps_mps;

	// ... and initialize states with IMU Data
	//pitchAngle_nav_rads = asin(xAcceleration_imu_mpsSq/g); // theta from Ax, aircraft at rest
	//rollAngle_nav_rads = asin(-yAcceleration_imu_mpsSq/(g*cos(pitchAngle_nav_rads))); // phi from Ay, aircraft at rest
	pitchAngle_nav_rads = 8*D2R;
	rollAngle_nav_rads = 0*D2R;
	yawAngle_nav_rads = 90.0*D2R;

	eul2quat(quat_nav,rollAngle_nav_rads,pitchAngle_nav_rads,yawAngle_nav_rads);

	accelerometerBias_nav_mpsSq[0] = 0.0;
	accelerometerBias_nav_mpsSq[1] = 0.0;
	accelerometerBias_nav_mpsSq[2] = 0.0;

	gyroBias_nav_rps[0] = p_imu_rps;
	gyroBias_nav_rps[1] = q_imu_rps;
	gyroBias_nav_rps[2] = r_imu_rps;

	// Specific forces and Rotation Rate
	f_b[0][0] = xAcceleration_imu_mpsSq - accelerometerBias_nav_mpsSq[0];
	f_b[1][0] = yAcceleration_imu_mpsSq - accelerometerBias_nav_mpsSq[1];
	f_b[2][0] = zAcceleration_imu_mpsSq - accelerometerBias_nav_mpsSq[2];

	om_ib[0][0] = p_imu_rps - gyroBias_nav_rps[0];
	om_ib[1][0] = q_imu_rps - gyroBias_nav_rps[1];
	om_ib[2][0] = r_imu_rps - gyroBias_nav_rps[2];

	// Time during initialization
	tprev = time_imu_sec;

	//Set output variables back to SGProps format
	latitude_nav_rads_node->setDoubleValue(latitude_nav_rads); 			///< [rad], geodetic latitude estimate
	longitude_nav_rads_node->setDoubleValue(longitude_nav_rads);			///< [rad], geodetic longitude estimate
	altitude_nav_m_node->setDoubleValue(altitude_nav_m);				///< [m], altitude relative to WGS84 estimate
	northVelocity_nav_mps_node->setDoubleValue(northVelocity_nav_mps); 		///< [m/sec], north velocity estimate
	eastVelocity_nav_mps_node->setDoubleValue(eastVelocity_nav_mps); 		///< [m/sec], east velocity estimate
	downVelocity_nav_mps_node->setDoubleValue(downVelocity_nav_mps); 		///< [m/sec], down velocity estimate
	rollAngle_nav_rads_node->setDoubleValue(rollAngle_nav_rads); 		///< [rad], Euler roll angle estimate
	pitchAngle_nav_rads_node->setDoubleValue(pitchAngle_nav_rads); 		///< [rad], Euler pitch angle estimate
	yawAngle_nav_rads_node->setDoubleValue(yawAngle_nav_rads); 			///< [rad], Euler yaw angle estimate
	///< Quaternions estimate
	quat_nav_node[0]->setDoubleValue(quat_nav[0]);
	quat_nav_node[1]->setDoubleValue(quat_nav[1]);
	quat_nav_node[2]->setDoubleValue(quat_nav[2]);
	quat_nav_node[3]->setDoubleValue(quat_nav[3]);
	///< [m/sec^2], accelerometer bias estimate
	accelerometerBias_nav_mpsSq_node[0]->setDoubleValue(accelerometerBias_nav_mpsSq[0]);
	accelerometerBias_nav_mpsSq_node[1]->setDoubleValue(accelerometerBias_nav_mpsSq[1]);
	accelerometerBias_nav_mpsSq_node[2]->setDoubleValue(accelerometerBias_nav_mpsSq[2]);
	///< [rad/sec], rate gyro bias estimate
	gyroBias_nav_rps_node[0]->setDoubleValue(gyroBias_nav_rps[0]);
	gyroBias_nav_rps_node[1]->setDoubleValue(gyroBias_nav_rps[1]);
	gyroBias_nav_rps_node[2]->setDoubleValue(gyroBias_nav_rps[2]);
	///< [rad], covariance estimate for position
	covariancePosition_nav_rads_node[0]->setDoubleValue(covariancePosition_nav_rads[0]);
	covariancePosition_nav_rads_node[1]->setDoubleValue(covariancePosition_nav_rads[1]);
	covariancePosition_nav_rads_node[2]->setDoubleValue(covariancePosition_nav_rads[2]);
	///< [rad], covariance estimate for velocity
	covarianceVelocity_nav_rads_node[0]->setDoubleValue(covarianceVelocity_nav_rads[0]);
	covarianceVelocity_nav_rads_node[1]->setDoubleValue(covarianceVelocity_nav_rads[1]);
	covarianceVelocity_nav_rads_node[2]->setDoubleValue(covarianceVelocity_nav_rads[2]);
	///< [rad], covariance estimate for angles
	covarianceAngles_nav_rads_node[0]->setDoubleValue(covarianceAngles_nav_rads[0]);
	covarianceAngles_nav_rads_node[1]->setDoubleValue(covarianceAngles_nav_rads[1]);
	covarianceAngles_nav_rads_node[2]->setDoubleValue(covarianceAngles_nav_rads[2]);
	///< [rad], covariance estimate for accelerometer bias
	covarianceAccelBias_nav_rads_node[0]->setDoubleValue(covarianceAccelBias_nav_rads[0]);
	covarianceAccelBias_nav_rads_node[1]->setDoubleValue(covarianceAccelBias_nav_rads[1]);
	covarianceAccelBias_nav_rads_node[2]->setDoubleValue(covarianceAccelBias_nav_rads[2]);
	///< [rad], covariance estimate for rate gyro bias
	covarianceGyroBias_nav_rads_node[0]->setDoubleValue(covarianceGyroBias_nav_rads[0]);
	covarianceGyroBias_nav_rads_node[1]->setDoubleValue(covarianceGyroBias_nav_rads[1]);
	covarianceGyroBias_nav_rads_node[2]->setDoubleValue(covarianceGyroBias_nav_rads[2]);

	err_type_nav = data_valid;

	err_type_nav_node->setIntValue(err_type_nav);
	//send_status("NAV filter initialized");
	//fprintf(stderr,"NAV Data Initialization Completed\n");
}

// Main get_nav filter function
void get_nav(){
	double tnow, imu_dt;
	double dq[4], quat_new[4];

	// compute time-elapsed 'dt'
	// This compute the navigation state at the DAQ's Time Stamp
	tnow = time_imu_sec;
	imu_dt = tnow - tprev;
	tprev = tnow;

	// ==================  Time Update  ===================
	// Temporary storage in Matrix form
	quat[0] = quat_nav[0];
	quat[1] = quat_nav[1];
	quat[2] = quat_nav[2];
	quat[3] = quat_nav[3];

	a_temp31[0][0] = northVelocity_nav_mps; a_temp31[1][0] = eastVelocity_nav_mps;
	a_temp31[2][0] = downVelocity_nav_mps;

	b_temp31[0][0] = latitude_nav_rads; b_temp31[1][0] = longitude_nav_rads;
	b_temp31[2][0] = altitude_nav_m;

	// AHRS Transformations
	C_N2B = quat2dcm(quat, C_N2B);
	C_B2N = mat_tran(C_N2B, C_B2N);

	// Attitude Update
	// ... Calculate Navigation Rate
	nr = navrate(a_temp31,b_temp31,nr);

	dq[0] = 1;
	dq[1] = 0.5*om_ib[0][0]*imu_dt;
	dq[2] = 0.5*om_ib[1][0]*imu_dt;
	dq[3] = 0.5*om_ib[2][0]*imu_dt;

	qmult(quat,dq,quat_new);

	quat[0] = quat_new[0]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
	quat[1] = quat_new[1]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
	quat[2] = quat_new[2]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
	quat[3] = quat_new[3]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);

    if(quat[0] < 0) {
        // Avoid quaternion flips sign
        quat[0] = -quat[0];
        quat[1] = -quat[1];
        quat[2] = -quat[2];
        quat[3] = -quat[3];
    }

	quat_nav[0] = quat[0];
	quat_nav[1] = quat[1];
	quat_nav[2] = quat[2];
	quat_nav[3] = quat[3];

	quat2eul(quat_nav,&(rollAngle_nav_rads),&(pitchAngle_nav_rads),&(yawAngle_nav_rads));

	// Velocity Update
	dx = mat_mul(C_B2N,f_b,dx);
	dx = mat_add(dx,grav,dx);
	northVelocity_nav_mps += imu_dt*dx[0][0];
	eastVelocity_nav_mps += imu_dt*dx[1][0];
	downVelocity_nav_mps += imu_dt*dx[2][0];

	// Position Update
	dx = llarate(a_temp31,b_temp31,dx);
	latitude_nav_rads += imu_dt*dx[0][0];
	longitude_nav_rads += imu_dt*dx[1][0];
	altitude_nav_m += imu_dt*dx[2][0];

	// JACOBIAN
	F = mat_fill(F, ZERO_MATRIX);
	// ... pos2gs
	F[0][3] = 1.0; 	F[1][4] = 1.0; 	F[2][5] = 1.0;
	// ... gs2pos
	F[5][2] = -2*g/EARTH_RADIUS;

	// ... gs2att
	temp33 = sk(f_b,temp33);
	atemp33 = mat_mul(C_B2N,temp33,atemp33);

	F[3][6] = -2.0*atemp33[0][0]; F[3][7] = -2.0*atemp33[0][1]; F[3][8] = -2.0*atemp33[0][2];
	F[4][6] = -2.0*atemp33[1][0]; F[4][7] = -2.0*atemp33[1][1]; F[4][8] = -2.0*atemp33[1][2];
	F[5][6] = -2.0*atemp33[2][0]; F[5][7] = -2.0*atemp33[2][1]; F[5][8] = -2.0*atemp33[2][2];

	// ... gs2acc
	F[3][9] = -C_B2N[0][0]; F[3][10] = -C_B2N[0][1]; F[3][11] = -C_B2N[0][2];
	F[4][9] = -C_B2N[1][0]; F[4][10] = -C_B2N[1][1]; F[4][11] = -C_B2N[1][2];
	F[5][9] = -C_B2N[2][0]; F[5][10] = -C_B2N[2][1]; F[5][11] = -C_B2N[2][2];

	// ... att2att
	temp33 = sk(om_ib,temp33);
	F[6][6] = -temp33[0][0]; F[6][7] = -temp33[0][1]; F[6][8] = -temp33[0][2];
	F[7][6] = -temp33[1][0]; F[7][7] = -temp33[1][1]; F[7][8] = -temp33[1][2];
	F[8][6] = -temp33[2][0]; F[8][7] = -temp33[2][1]; F[8][8] = -temp33[2][2];

	// ... att2gyr
	F[6][12] = -0.5;
	F[7][13] = -0.5;
	F[8][14] = -0.5;

	// ... Accel Markov Bias
	F[9][9] = -1.0/TAU_A; 	F[10][10] = -1.0/TAU_A;	F[11][11] = -1.0/TAU_A;
	F[12][12] = -1.0/TAU_G; F[13][13] = -1.0/TAU_G;	F[14][14] = -1.0/TAU_G;

	//fprintf(stderr,"Jacobian Created\n");

	// State Transition Matrix: PHI = I15 + F*dt;
	temp1515 = mat_scalMul(F,imu_dt,temp1515);
	PHI = mat_add(I15,temp1515,PHI);

	// Process Noise
	G = mat_fill(G, ZERO_MATRIX);
	G[3][0] = -C_B2N[0][0];	G[3][1] = -C_B2N[0][1]; G[3][2] = -C_B2N[0][2];
	G[4][0] = -C_B2N[1][0];	G[4][1] = -C_B2N[1][1]; G[4][2] = -C_B2N[1][2];
	G[5][0] = -C_B2N[2][0];	G[5][1] = -C_B2N[2][1]; G[5][2] = -C_B2N[2][2];

	G[6][3] = -0.5;
	G[7][4] = -0.5;
	G[8][5] = -0.5;

	G[9][6] = 1.0; 			G[10][7] = 1.0; 		G[11][8] = 1.0;
	G[12][9] = 1.0; 		G[13][10] = 1.0; 		G[14][11] = 1.0;
	//fprintf(stderr,"Process Noise Matrix G is created\n");
	// Discrete Process Noise
	temp1512 = mat_mul(G,Rw,temp1512);
	temp1515 = mat_transmul(temp1512,G,temp1515);	// Qw = G*Rw*G'
	Qw = mat_scalMul(temp1515,imu_dt,Qw);			// Qw = dt*G*Rw*G'
	Q = mat_mul(PHI,Qw,Q);						// Q = (I+F*dt)*Qw

	temp1515 = mat_tran(Q,temp1515);
	Q = mat_add(Q,temp1515,Q);
	Q = mat_scalMul(Q,0.5,Q);				// Q = 0.5*(Q+Q')
	//fprintf(stderr,"Discrete Process Noise is created\n");

	// Covariance Time Update
	temp1515 = mat_mul(PHI,P,temp1515);
	P = mat_transmul(temp1515,PHI,P); 		// P = PHI*P*PHI'
	P = mat_add(P,Q,P);						// P = PHI*P*PHI' + Q
	temp1515 = mat_tran(P, temp1515);
	P = mat_add(P,temp1515,P);
	P = mat_scalMul(P,0.5,P);				// P = 0.5*(P+P')
	//fprintf(stderr,"Covariance Updated through Time Update\n");

	covariancePosition_nav_rads[0] = P[0][0]; covariancePosition_nav_rads[1] = P[1][1]; covariancePosition_nav_rads[2] = P[2][2];
	covarianceVelocity_nav_rads[0] = P[3][3]; covarianceVelocity_nav_rads[1] = P[4][4]; covarianceVelocity_nav_rads[2] = P[5][5];
	covarianceAngles_nav_rads[0] = P[6][6]; covarianceAngles_nav_rads[1] = P[7][7]; covarianceAngles_nav_rads[2] = P[8][8];
	covarianceAccelBias_nav_rads[0] = P[9][9]; covarianceAccelBias_nav_rads[1] = P[10][10]; covarianceAccelBias_nav_rads[2] = P[11][11];
	covarianceGyroBias_nav_rads[0] = P[12][12]; covarianceGyroBias_nav_rads[1] = P[13][13]; covarianceGyroBias_nav_rads[2] = P[14][14];

	err_type_nav = TU_only;
	//fprintf(stderr,"Time Update Done\n");
	// ==================  DONE TU  ===================

	if(newData){

		// ==================  GPS Update  ===================
		newData = 0; // Reset the flag

		// Position, converted to NED
		a_temp31[0][0] = latitude_nav_rads;
		a_temp31[1][0] = longitude_nav_rads; a_temp31[2][0] = altitude_nav_m;
		pos_ins_ecef = lla2ecef(a_temp31,pos_ins_ecef);

		a_temp31[2][0] = 0.0;
		//pos_ref = lla2ecef(a_temp31,pos_ref);
		pos_ref = mat_copy(a_temp31,pos_ref);
		pos_ins_ned = ecef2ned(pos_ins_ecef,pos_ins_ned,pos_ref);

		pos_gps[0][0] = latitude_gps_degs*D2R;
		pos_gps[1][0] = longitude_gps_degs*D2R;
		pos_gps[2][0] = altitude_gps_m;

		pos_gps_ecef = lla2ecef(pos_gps,pos_gps_ecef);

		pos_gps_ned = ecef2ned(pos_gps_ecef,pos_gps_ned,pos_ref);

		// Create Measurement: y
		y[0][0] = pos_gps_ned[0][0] - pos_ins_ned[0][0];
		y[1][0] = pos_gps_ned[1][0] - pos_ins_ned[1][0];
		y[2][0] = pos_gps_ned[2][0] - pos_ins_ned[2][0];

		y[3][0] = northVelocity_gps_mps - northVelocity_nav_mps;
		y[4][0] = eastVelocity_gps_mps - eastVelocity_nav_mps;
		y[5][0] = downVelocity_gps_mps - downVelocity_nav_mps;

		//fprintf(stderr,"Measurement Matrix, y, created\n");

		// Kalman Gain
		temp615 = mat_mul(H,P,temp615);
		temp66 = mat_transmul(temp615,H,temp66);
		atemp66 = mat_add(temp66,R,atemp66);
		temp66 = mat_inv(atemp66,temp66); // temp66 = inv(H*P*H'+R)
		//fprintf(stderr,"inv(H*P*H'+R) Computed\n");

		temp156 = mat_transmul(P,H,temp156); // P*H'
		//fprintf(stderr,"P*H' Computed\n");
		K = mat_mul(temp156,temp66,K);	   // K = P*H'*inv(H*P*H'+R)
		//fprintf(stderr,"Kalman Gain Computed\n");

		// Covariance Update
		temp1515 = mat_mul(K,H,temp1515);
		ImKH = mat_sub(I15,temp1515,ImKH);	// ImKH = I - K*H

		temp615 = mat_transmul(R,K,temp615);
		KRKt = mat_mul(K,temp615,KRKt);		// KRKt = K*R*K'

		temp1515 = mat_transmul(P,ImKH,temp1515);
		P = mat_mul(ImKH,temp1515,P);		// ImKH*P*ImKH'
		temp1515 = mat_add(P,KRKt,temp1515);
		P = mat_copy(temp1515,P);			// P = ImKH*P*ImKH' + KRKt
		//fprintf(stderr,"Covariance Updated through GPS Update\n");

		covariancePosition_nav_rads[0] = P[0][0]; covariancePosition_nav_rads[1] = P[1][1]; covariancePosition_nav_rads[2] = P[2][2];
		covarianceVelocity_nav_rads[0] = P[3][3]; covarianceVelocity_nav_rads[1] = P[4][4]; covarianceVelocity_nav_rads[2] = P[5][5];
		covarianceAngles_nav_rads[0] = P[6][6]; covarianceAngles_nav_rads[1] = P[7][7]; covarianceAngles_nav_rads[2] = P[8][8];
		covarianceAccelBias_nav_rads[0] = P[9][9]; covarianceAccelBias_nav_rads[1] = P[10][10]; covarianceAccelBias_nav_rads[2] = P[11][11];
		covarianceGyroBias_nav_rads[0] = P[12][12]; covarianceGyroBias_nav_rads[1] = P[13][13]; covarianceGyroBias_nav_rads[2] = P[14][14];

		// State Update
		x = mat_mul(K,y,x);
		denom = (1.0 - (ECC2 * sin(latitude_nav_rads) * sin(latitude_nav_rads)));
		denom = sqrt(denom*denom);

		Re = EARTH_RADIUS / sqrt(denom);
		Rn = EARTH_RADIUS*(1-ECC2) / denom*sqrt(denom);
		altitude_nav_m = altitude_nav_m - x[2][0];
		latitude_nav_rads = latitude_nav_rads + x[0][0]/(Re + altitude_nav_m);
		longitude_nav_rads = longitude_nav_rads + x[1][0]/(Rn + altitude_nav_m)/cos(latitude_nav_rads);

		northVelocity_nav_mps = northVelocity_nav_mps + x[3][0];
		eastVelocity_nav_mps = eastVelocity_nav_mps + x[4][0];
		downVelocity_nav_mps = downVelocity_nav_mps + x[5][0];

		quat[0] = quat_nav[0];
		quat[1] = quat_nav[1];
		quat[2] = quat_nav[2];
		quat[3] = quat_nav[3];

		// Attitude correction
		dq[0] = 1.0;
		dq[1] = x[6][0];
		dq[2] = x[7][0];
		dq[3] = x[8][0];

		qmult(quat,dq,quat_new);

		quat[0] = quat_new[0]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
		quat[1] = quat_new[1]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
		quat[2] = quat_new[2]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);
		quat[3] = quat_new[3]/sqrt(quat_new[0]*quat_new[0] + quat_new[1]*quat_new[1] + quat_new[2]*quat_new[2] + quat_new[3]*quat_new[3]);

		quat_nav[0] = quat[0];
		quat_nav[1] = quat[1];
		quat_nav[2] = quat[2];
		quat_nav[3] = quat[3];

		quat2eul(quat_nav,&(rollAngle_nav_rads),&(pitchAngle_nav_rads),&(yawAngle_nav_rads));

		accelerometerBias_nav_mpsSq[0] = accelerometerBias_nav_mpsSq[0] + x[9][0];
		accelerometerBias_nav_mpsSq[1] = accelerometerBias_nav_mpsSq[1] + x[10][0];
		accelerometerBias_nav_mpsSq[2] = accelerometerBias_nav_mpsSq[2] + x[11][0];

		gyroBias_nav_rps[0] = gyroBias_nav_rps[0] + x[12][0];
		gyroBias_nav_rps[1] = gyroBias_nav_rps[1] + x[13][0];
		gyroBias_nav_rps[2] = gyroBias_nav_rps[2] + x[14][0];

		err_type_nav = gps_aided;
		//fprintf(stderr,"Measurement Update Done\n");
	}

	// Remove current estimated biases from rate gyro and accels
	p_imu_rps -= gyroBias_nav_rps[0];
	q_imu_rps -= gyroBias_nav_rps[1];
	r_imu_rps -= gyroBias_nav_rps[2];
	xAcceleration_imu_mpsSq -= accelerometerBias_nav_mpsSq[0];
	yAcceleration_imu_mpsSq -= accelerometerBias_nav_mpsSq[1];
	zAcceleration_imu_mpsSq -= accelerometerBias_nav_mpsSq[2];

	// Get the new Specific forces and Rotation Rate,
	// use in the next time update
	f_b[0][0] = xAcceleration_imu_mpsSq;
	f_b[1][0] = yAcceleration_imu_mpsSq;
	f_b[2][0] = zAcceleration_imu_mpsSq;

	om_ib[0][0] = p_imu_rps;
	om_ib[1][0] = q_imu_rps;
	om_ib[2][0] = r_imu_rps;

	//Set output variables back to SGProps format
	latitude_nav_rads_node->setDoubleValue(latitude_nav_rads); 			///< [rad], geodetic latitude estimate
	longitude_nav_rads_node->setDoubleValue(longitude_nav_rads);			///< [rad], geodetic longitude estimate
	altitude_nav_m_node->setDoubleValue(altitude_nav_m);				///< [m], altitude relative to WGS84 estimate
	northVelocity_nav_mps_node->setDoubleValue(northVelocity_nav_mps); 		///< [m/sec], north velocity estimate
	eastVelocity_nav_mps_node->setDoubleValue(eastVelocity_nav_mps); 		///< [m/sec], east velocity estimate
	downVelocity_nav_mps_node->setDoubleValue(downVelocity_nav_mps); 		///< [m/sec], down velocity estimate
	///< Quaternions estimate
	quat_nav_node[0]->setDoubleValue(quat_nav[0]);
	quat_nav_node[1]->setDoubleValue(quat_nav[1]);
	quat_nav_node[2]->setDoubleValue(quat_nav[2]);
	quat_nav_node[3]->setDoubleValue(quat_nav[3]);
	///< [m/sec^2], accelerometer bias estimate
	accelerometerBias_nav_mpsSq_node[0]->setDoubleValue(accelerometerBias_nav_mpsSq[0]);
	accelerometerBias_nav_mpsSq_node[1]->setDoubleValue(accelerometerBias_nav_mpsSq[1]);
	accelerometerBias_nav_mpsSq_node[2]->setDoubleValue(accelerometerBias_nav_mpsSq[2]);
	///< [rad/sec], rate gyro bias estimate
	gyroBias_nav_rps_node[0]->setDoubleValue(gyroBias_nav_rps[0]);
	gyroBias_nav_rps_node[1]->setDoubleValue(gyroBias_nav_rps[1]);
	gyroBias_nav_rps_node[2]->setDoubleValue(gyroBias_nav_rps[2]);
	///< [rad], covariance estimate for position
	covariancePosition_nav_rads_node[0]->setDoubleValue(covariancePosition_nav_rads[0]);
	covariancePosition_nav_rads_node[1]->setDoubleValue(covariancePosition_nav_rads[1]);
	covariancePosition_nav_rads_node[2]->setDoubleValue(covariancePosition_nav_rads[2]);
	///< [rad], covariance estimate for velocity
	covarianceVelocity_nav_rads_node[0]->setDoubleValue(covarianceVelocity_nav_rads[0]);
	covarianceVelocity_nav_rads_node[1]->setDoubleValue(covarianceVelocity_nav_rads[1]);
	covarianceVelocity_nav_rads_node[2]->setDoubleValue(covarianceVelocity_nav_rads[2]);
	///< [rad], covariance estimate for angles
	covarianceAngles_nav_rads_node[0]->setDoubleValue(covarianceAngles_nav_rads[0]);
	covarianceAngles_nav_rads_node[1]->setDoubleValue(covarianceAngles_nav_rads[1]);
	covarianceAngles_nav_rads_node[2]->setDoubleValue(covarianceAngles_nav_rads[2]);
	///< [rad], covariance estimate for accelerometer bias
	covarianceAccelBias_nav_rads_node[0]->setDoubleValue(covarianceAccelBias_nav_rads[0]);
	covarianceAccelBias_nav_rads_node[1]->setDoubleValue(covarianceAccelBias_nav_rads[1]);
	covarianceAccelBias_nav_rads_node[2]->setDoubleValue(covarianceAccelBias_nav_rads[2]);
	///< [rad], covariance estimate for rate gyro bias
	covarianceGyroBias_nav_rads_node[0]->setDoubleValue(covarianceGyroBias_nav_rads[0]);
	covarianceGyroBias_nav_rads_node[1]->setDoubleValue(covarianceGyroBias_nav_rads[1]);
	covarianceGyroBias_nav_rads_node[2]->setDoubleValue(covarianceGyroBias_nav_rads[2]);
	err_type_nav_node->setIntValue(err_type_nav);

}

void close_nav(void){
	//free memory space
	mat_free(F);
	mat_free(PHI);
	mat_free(P);
	mat_free(G);
	mat_free(Rw);
	mat_free(Q);
	mat_free(Qw);
	mat_free(R);
	mat_free(eul);
	mat_free(x);
	mat_free(y);
	mat_free(Rbodtonav);
	mat_free(grav);
	mat_free(f_b);
	mat_free(om_ib);
	mat_free(nr);
	mat_free(K);
	mat_free(H);
	mat_free(C_N2B);
	mat_free(C_B2N);
	mat_free(pos_ref);
	mat_free(pos_ins_ecef);
	mat_free(pos_ins_ned);
	mat_free(pos_gps);
	mat_free(pos_gps_ecef);
	mat_free(pos_gps_ned);
	mat_free(I15);
	mat_free(I3);
	mat_free(ImKH);
	mat_free(KRKt);
	mat_free(dx);
	mat_free(a_temp31);
	mat_free(b_temp31);
	mat_free(temp33);
	mat_free(atemp33);
	mat_free(temp1515);
	mat_free(temp615);
	mat_free(temp66);
	mat_free(atemp66);
	mat_free(temp156);
	mat_free(temp1512);

}
