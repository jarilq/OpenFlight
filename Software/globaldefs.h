/*! \file globaldefs.h
 *	\brief Global definitions
 *
 *	\details This file is used to define macros and structures used in the program
 *
 * \author University of Minnesota
 * \author Aerospace Engineering and Mechanics
 * \copyright Copyright 2011 Regents of the University of Minnesota. All rights reserved.
 *
 * $Id: globaldefs_new_signals.h 854 2012-07-10 13:32:52Z joh07594 $
 */
#ifndef GLOBALDEFS_H_
#define GLOBALDEFS_H_

// ****** Macro function definitions *******************************************
#define	mymin(arg1,arg2) 	(arg1<=arg2 ? arg1:arg2) 	///< Return the lesser of two input arguments */
#define	mymax(arg1,arg2)	(arg1>=arg2 ? arg1:arg2)	///< Return the greater of two input arguments */
#define sign(arg) 			(arg>=0 ? 1:-1) 			///< Return the sign of the input argument */
// *****************************************************************************

// ******  Thread Settings *****************************************************
#define TIMESTEP 0.02 ///< Base time step, needed for control laws */
// *****************************************************************************

// ****** Unit conversions and constant definitions: ***************************
#define NSECS_PER_SEC	1000000000 		///< [nsec/sec] nanoseconds per second */
#define D2R			0.017453292519943	///< [rad] degrees to radians */
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
// *****************************************************************************

// ****** Type definitions *****************************************************
typedef unsigned char	byte;	///< typedef of byte */
typedef unsigned short	word;	///< typedef of word */

/// Define status message enum list
enum   errdefs	{
	got_invalid,		///< No data received
	checksum_err,		///< Checksum mismatch
	gps_nolock,			///< No GPS lock
	data_valid,			///< Data valid
	noPacketHeader,		///< Some data received, but cannot find packet header
	incompletePacket,	///< Packet header found, but complete packet not received
	TU_only,			///< NAV filter, time update only
	gps_aided,			///< NAV filter, GPS aided
	};

/// IMU Data Structure
struct imu {
	double p_imu_rps;	///< [rad/sec], body X axis angular rate (roll)
	double q_imu_rps;	///< [rad/sec], body Y axis angular rate (pitch)
	double r_imu_rps;	///< [rad/sec], body Z axis angular rate (yaw)

	double ax_imu_mpss;	///< [m/sec^2], body X axis acceleration
	double ay_imu_mpss;	///< [m/sec^2], body Y axis acceleration
	double az_imu_mpss;	///< [m/sec^2], body Z axis acceleration

	double hx_imu_Gs;	///< [Gauss], body X axis magnetic field
	double hy_imu_Gs;	///< [Gauss], body Y axis magnetic field
	double hz_imu_Gs;	///< [Gauss], body Z axis magnetic field

	double phi_imu_rad; ///< [rad], Euler roll angle. Only used if IMU sensor reports attitude.
	double theta_imu_rad; ///< [rad], Euler pitch angle. Only used if IMU sensor reports attitude.
	double psi_imu_rad; ///< [rad], Euler yaw angle. Only used if IMU sensor reports attitude.

	float  temp_imu_C;	///< [degC], temperature of IMU sensor
	float  voltage_imu_V;	///< [Volts], supply voltage of IMU sensor

	enum errdefs err_type_imu; ///< IMU status
	double time_imu_s; ///< [sec], timestamp of IMU data
};

/// GPS Data Structure
struct gps {
	double lat_gps_deg;	///< [deg], Geodetic latitude
	double lon_gps_deg;	///< [deg], Geodetic longitude
	double altWGS84_gps_m;	///< [m], altitude relative to WGS84
	double ve_gps_mps;	///< [m/sec], East velocity
	double vn_gps_mps;	///< [m/sec], North velocity
	double vd_gps_mps;	///< [m/sec], Down velocity

	double xECEF_gps_m;	///< [m], X position, ECEF--means earth-centered earth fixed (coord system)
	double yECEF_gps_m;	///< [m], Y position, ECEF
	double zECEF_GPS_m;	///< [m], Z position, ECEF

	double vxECEF_gps_mps;	///< [m/sec], X velocity, ECEF
	double vyECEF_gps_mps;	///< [m/sec], Y velocity, ECEF
	double vzECEF_gps_mps;	///< [m/sec], Z velocity, ECEF

	double track_gps_rad;///< [rad], course over the ground, relative to true North
	double groundSpeed_gps_mps;	///< [rad]--mps?, speed over the ground

	double time_gps_s;	///< [sec], timestamp of GPS data

	unsigned short newData_gps_bool;	///< [bool], flag set when GPS data has been updated
	unsigned short satVisible_gps_nd; ///< Number satellites used in the position solution
	unsigned short navValid_gps_bool;///< flag indicating whether the solution is valid, 0 = valid

	enum errdefs err_type_gps;	///< GPS status
};

/// Air Data Structure
struct airdata {
	double vias_airdata_mps;	///< [m], barometric altitude above ground level (AGL)
	double vtrue_airdata_mps;     ///< [m/sec], indicated airspeed

	double Ps_airdata_kPa;		///< [KPa], static pressure
	double Pd_airdata_kPa;		///< [KPa], dynamic pressure

};

/// Pilot inceptor Data structure
struct inceptor {
	double throttleCmd_inceptor_nd;	///< throttle stick command from the pilot, ND
	double pitchCmd_inceptor_nd;		///< pitch stick command from the pilot, ND
	double yawCmd_inceptor_nd;			///< yaw stick command from the pilot, ND
	double rollCmd_inceptor_nd;		///< roll stick command from the pilot, ND
	double mode_inceptor_nd;		//added to run with mAEWing1 mission code
	double select_inceptor_nd;		//added to run with mAEWing1 mission code
};

/// Mission manager Data structure
struct mission {
	unsigned short mode_mission_nd;			///< mode variable; 0 = dump data, 1 = manual control, 2 = autopilot control
	unsigned short runNum_mission_nd;		///< counter for number of autopilot engagements
	unsigned short clawMode_mission_nd;	//added to run with mAEWing1 mission code
	unsigned short clawSelect_mission_nd;	//added to run with mAEWing1 mission code
};

/// Control Data structure
struct control {
	double thetaCmd_ctrl_rad;	///< [rad], Euler pitch angle command
	double psiCmd_ctrl_rad;		///< [rad], Euler yaw angle command
	double phiCmd_ctrl_rad;		///< [rad], Euler roll angle command

	double pCmd_ctrl_rps;		///< [rad/sec], body axis roll rate command
	double qCmd_ctrl_rps;		///< [rad/sec], body axis pitch rate command
	double rCmd_ctrl_rps;		///< [rad/sec], body axis yaw rate command
	double viasCmd_ctrl_mps;		///< [m/sec], airspeed command
	double altAGLcmd_ctrl_m;		///< [m], altitude command
	double gndtrkCmd_ctrl_rad;	///< [rad], ground track angle command, relative to true north
	double aoaCmd_ctrl_rad;		///< [rad], angle of attack command
	double aosCmd_ctrl_rad;		///< [rad], angle of sideslip command
	double gammaCmd_ctrl_rad;	///< [rad], flight path angle command
};

/// Navigation Filter Data Structure
struct nav {
	double lat_nav_rad;		///< [rad], geodetic latitude estimate
	double lon_nav_rad;		///< [rad], geodetic longitude estimate
	double alt_nav_m;			///< [m], altitude relative to WGS84 estimate
	double vn_nav_mps;			///< [m/sec], north velocity estimate
	double ve_nav_mps;			///< [m/sec], east velocity estimate
	double vd_nav_mps;			///< [m/sec], down velocity estimate

	double phi_nav_rad;		///< [rad], Euler roll angle estimate
	double theta_nav_rad;		///< [rad], Euler pitch angle estimate
	double psi_nav_rad;		///< [rad], Euler yaw angle estimate

	enum errdefs err_type_nav;	///< NAV filter status
	double time_nav_s;			///< [sec], timestamp of NAV filter

	double wn_nav_mps;			///< [m/s], estimated wind speed in the north direction
	double we_nav_mps;			///< [m/s], estimated wind speed in the east direction
	double wd_nav_mps;			///< [m/s], estimated wind speed in the down direction
};

/// Combined sensor data structure
struct sensordata {
	struct imu *imuData_ptr; 			///< pointer to imu data structure
	struct gps *gpsData_ptr;			///< pointer to gps data structure
	struct airdata *airData_ptr;			///< pointer to airdata data structure
	struct surface *surfData_ptr;		///< pointer to surface data structure
	struct inceptor *inceptorData_ptr;	///< pointer to pilot inceptor data structure
};

/// Datalogging data structure
struct datalog {
	char** saveAsDoubleNames_dtlog_ptr;		///< pointer to char array of variable names for doubles
	double** saveAsDoublePointers_dtlog_ptr;	///< pointer to double pointer array to variables that will be saved as doubles
	char** saveAsFloatNames_dtlog_ptr;		///< pointer to char array of variable names for floats
	double** saveAsFloatPointers_dtlog_ptr;	///< pointer to double pointer array to variables that will be saved as floats
	char** saveAsIntNames_dtlog_ptr;			///< pointer to char array of variable names for ints
	int** saveAsIntPointers_dtlog_ptr;		///< pointer to int32_t pointer array to variables that will be saved as ints
	char** saveAsShortNames_dtlog_ptr;		///< pointer to char array of variable names for shorts
	unsigned short** saveAsShortPointers_dtlog_ptr;	///< pointer to uint16_t pointer array to variables that will be saved as shorts
	int logArraySize_dtlog_nu; 	///< Number of data points in the logging array. 50 Hz * 60 sec/min * 30 minutes = 90000
	int numDoubleVars_dtlog_nu;	///< Number of variables that will be logged as doubles
	int numFloatVars_dtlog_nu;	///< Number of variables that will be logged as floats
	int numIntVars_dtlog_nu;		///< Number of variables that will be logged as ints
	int numShortVars_dtlog_nu;	///< Number of variables that will be logged as shorts
};
#endif	/* GLOBALDEFS_H_ */
