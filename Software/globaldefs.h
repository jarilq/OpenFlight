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
	double xAcceleration_imu_mpsSq;	///< [m/sec^2], body X axis acceleration
	double yAcceleration_imu_mpsSq;	///< [m/sec^2], body Y axis acceleration
	double zAcceleration_imu_mpsSq;	///< [m/sec^2], body Z axis acceleration
	double x_imu_gau;	///< [Gauss], body X axis magnetic field
	double y_imu_gau;	///< [Gauss], body Y axis magnetic field
	double z-imu_gau;	///< [Gauss], body Z axis magnetic field
	double phi_imu_rads; ///< [rad], Euler roll angle. Only used if IMU sensor reports attitude.
	double the_imu_rads; ///< [rad], Euler pitch angle. Only used if IMU sensor reports attitude.
	double psi_imu_rads; ///< [rad], Euler yaw angle. Only used if IMU sensor reports attitude.
	float  T_imu_degC;	///< [degC], temperature of IMU sensor
	float  Vs_imu_vol;	///< [Volts], supply voltage of IMU sensor
	double adc_imu_cou; ///< [counts], ADC reading
	enum errdefs_imu_nu; ///< IMU status
	double time_imu_sec; ///< [sec], timestamp of IMU data
};

/// GPS Data Structure
struct gps {
	double latitude_gps_degs;	///< [deg], Geodetic latitude
	double longitude_gps_degs;	///< [deg], Geodetic longitude
	double altitude_gps_m;	///< [m], altitude relative to WGS84
	double eastVelocity_gps_mps;	///< [m/sec], East velocity
	double northVelocity_gps_mps;	///< [m/sec], North velocity
	double downVelocity_gps_mps;	///< [m/sec], Down velocity
	double Xe_gps_m;	///< [m], X position, ECEF
	double Ye_gps_m;	///< [m], Y position, ECEF
	double Ze_gps_m;	///< [m], Z position, ECEF
	double Ue_gps_mps;	///< [m/sec], X velocity, ECEF
	double Ve_gps_mps;	///< [m/sec], Y velocity, ECEF
	double We_gps_mps;	///< [m/sec], Z velocity, ECEF
    	double sigN_gps_m; ///< [m], Position error standard deviation in the North direction
	double sigE_gps_m; ///< [m], Position error standard deviation in the East direction
	double sigD_gps_m; ///< [m], Position error standard deviation in the Down direction
    	double sigvn_gps_mps; ///< [m/sec], Velocity error standard deviation in the North direction
    	double sigve_gps_mps; ///< [m/sec], Velocity error standard deviation in the East direction
    	double sigvd_gps_mps; ///< [m/sec], Velocity error standard deviation in the Down direction
	double TOW_gps_sec;	///< [sec], GPS Time Of Week
	double cogN_gps_rads;///< [rad], course over the ground, relative to true North
	double sog_gps_rads;	///< [rad], speed over the ground
	double time_gps_sec;	///< [sec], timestamp of GPS data
	unsigned short newData_gps_nd;	///< [bool], flag set when GPS data has been updated
	unsigned short satV_gps_nu; ///< Number satellites used in the position solution
	unsigned short nValid_gps_nd;///< flag indicating whether the solution is valid, 0 = valid
	unsigned short GPSw_gps_week;///< GPS week since current epoch.
	enum errdefs_gps_nu;	///< GPS status
	int bRate_gps_nu;		///< Baud rate for serial port
	char* portName_gps_nd;		///< Name of serial port
	int port_gps_nu;			///< handle for accessing serial port
	unsigned char* locBuffer_gps_nd; ///< local buffer to store partial serial data packets
	int bytLocBuffer_gps_bytes; ///< number of bytes in the local buffer
	int readState_gps_nu;			///< current state of serial data reader function
    	int readcalls_gps_nu;			///< number of times the read_gps function has been called

};

/// Air Data Structure
struct airdata {
	double h_airdata_m;		///< [m], barometric altitude above ground level (AGL)
	double ias_airdata_mps;     ///< [m/sec], indicated airspeed
	double hfilt_airdata_m;	///< [m], filtered altitude
	double iasfilt_airdata_mps;	///< [m/s], filtered airspeed
	double Ps_airdata_KPa;		///< [KPa], static pressure
	double Pd_airdata_KPa;		///< [KPa], dynamic pressure
	double aoa_airdata_rads;		///< [rad], angle of attack from 5-hole Pitot probe
	double aos_airdata_rads;		///< [rad], angle of sideslip from 5-hole Pitot probe
	double lalpha_airdata_rads; ///< [rad], angle of attack, from left vane
	double ralpha_airdata_rads;	///< [rad], angle of attack, from right vane
	double lbeta_airdata_rads;	///< [rad], angle of sideslip, from left vane
	double rbeta_airdata_rads;	///< [rad], angle of sideslip, from right vane
	double Pdaoa_airdata_KPa;  ///< [KPa], dynamic pressure for aoa, from 5-hole Pitot probe
	double Pdaos_airdata_KPa;	///< [KPa], dynamic pressure for aos, from 5-hole Pitot probe
	double bias_airdata[10];///< array for storing biases for air data.
	unsigned short status_airdata_nd;	///< status bitfield for air data sensors.
};

/// Control surface deflections
struct surface {
	double thropos_surface_nu;	///< [0-1], measured throttle position
	double elevpos_surface_rads;		///< [rad], measured elevator position, +TED
	double rudpos_surface_rads; 		///< [rad], measured rudder position, +TEL
	double lailpos_surface_rads;	///< [rad], measured left aileron position, +TED
	double railpos_surface_rads;	///< [rad], measured right aileron position, +TED
	double lflapos_surface_rads;	///< [rad], measured left flap position, +TED
	double rflapos_surface_rads;	///< [rad], measured right flap position, +TED
};

/// Pilot inceptor Data structure
struct inceptor {
	double thro_inceptor_nu;	///< throttle stick command from the pilot, ND
	double pit_inceptor_nu;		///< pitch stick command from the pilot, ND
	double yaw_inceptor_nu;			///< yaw stick command from the pilot, ND
	double rol_inceptor_nu;		///< roll stick command from the pilot, ND
	double mode_inceptor_nu;		//added to run with mAEWing1 mission code
	double select_inceptor_nu;		//added to run with mAEWing1 mission code
};

/// Mission manager Data structure
struct mission {
	unsigned short mode_mission_nd;		///< mode variable; 0 = dump data, 1 = manual control, 2 = autopilot control
	unsigned short rnum_mission_nu;		///< counter for number of autopilot engagements
	unsigned short reNav_mission_nd;	///< mode variable; 0 = standard nav filter, 1 = research nav filter
	unsigned short clawmode_mission_nd;		//added to run with mAEWing1 mission code
	unsigned short clawselect_mission_nd;		//added to run with mAEWing1 mission code
};

/// Control Data structure
struct control {
	double throcmd_control_nu;		///< [0-1], throttle command
	double elevcmd_control_rads;			///< [rad], elevator command, +TED
	double rudcmd_control_rads; 			///< [rad], rudder command, +TEL
	double lailcmd_control_rads;		///< [rad], left aileron command, +TED
	double railcmd_control_rads;		///< [rad], right aileron command, +TED
	double lflacmd_control_rads;		///< [rad], left flap command, +TED
	double rflacmd_control_rads;		///< [rad], right flap command, +TED
	double phicmd_control_rads;		///< [rad], Euler roll angle command
	double thetcmd_control_rads;	///< [rad], Euler pitch angle command
	double psicmd_control_rads;		///< [rad], Euler yaw angle command
	double pcmd_control_rps;		///< [rad/sec], body axis roll rate command
	double qcmd_control_rps;		///< [rad/sec], body axis pitch rate command
	double rcmd_control_rps;		///< [rad/sec], body axis yaw rate command
	double iascmd_control_mps;		///< [m/sec], airspeed command
	double hcmd_control_m;		///< [m], altitude command
	double gndtrkcmd_control_rads;	///< [rad], ground track angle command, relative to true north
	double aoacmd_control_rads;		///< [rad], angle of attack command
	double aoscmd_control_rads;		///< [rad], angle of sideslip command
	double gammacmd_control_rads;	///< [rad], flight path angle command
	double signal0_control_nu;     ///< user defined dummy variable
	double signal1_control_nu;     ///< user defined dummy variable
	double signal2_control_nu;     ///< user defined dummy variable
	double signal3_control_nu;     ///< user defined dummy variable
	double signal4_control_nu;     ///< user defined dummy variable
	double signal5_control_nu;     ///< user defined dummy variable
	double signal6_control_nu;     ///< user defined dummy variable
	double signal7_control_nu;     ///< user defined dummy variable
	double signal8_control_nu;     ///< user defined dummy variable
	double signal9_control_nu;     ///< user defined dummy variable
};

/// Navigation Filter Data Structure
struct nav {
	double latitude_nav_rads;		///< [rad], geodetic latitude estimate
	double longitude_nav_rads;		///< [rad], geodetic longitude estimate
	double altitude_nav_m;		///< [m], altitude relative to WGS84 estimate
	double northVelocity_nav_mps;		///< [m/sec], north velocity estimate
	double eastVelocity_nav_mps;		///< [m/sec], east velocity estimate
	double downVelocity_nav_mps;		///< [m/sec], down velocity estimate
	double rollAngle_nav_rads;		///< [rad], Euler roll angle estimate
	double pitchAngle_nav_rads;		///< [rad], Euler pitch angle estimate
	double yawAngle_nav_rads;		///< [rad], Euler yaw angle estimate
	double quat_nav_nu[4];	///< Quaternions estimate
	double accelerometerBias_nav_mpsSq[3];	///< [m/sec^2], accelerometer bias estimate
	double gyroBias_nav_rps[3];	///< [rad/sec], rate gyro bias estimate
	double asf_nav_mpsSq[3];	///< [m/sec^2], accelerometer scale factor estimate
	double gsf_nav_rps[3];	///< [rad/sec], rate gyro scale factor estimate
	double covariancePosition_nav_rads[3];	///< [rad], covariance estimate for position
	double covarianceVelocity_nav_rads[3];	///< [rad], covariance estimate for velocity
	double covarianceAngles_nav_rads[3];	///< [rad], covariance estimate for angles
	double covarianceAccelBias_nav_rads[3];	///< [rad], covariance estimate for accelerometer bias
	double covarianceGyroBias_nav_rads[3];	///< [rad], covariance estimate for rate gyro bias
	double Pasf_nav_rads[3];	///< [rad], covariance estimate for accelerometer scale factor
	double Pgsf_nav_rads[3];	///< [rad], covariance estimate for rate gyro scale factor
	enum errdefs_nav_nu;	///< NAV filter status
	double timestm_nav_sec;			///< [sec], timestamp of NAV filter
	double wn_nav_mps;			///< [m/s], estimated wind speed in the north direction
	double we_nav_mps;			///< [m/s], estimated wind speed in the east direction
	double wd_nav_mps;			///< [m/s], estimated wind speed in the down direction
	double signal0_nav_nu;     ///< user defined dummy variable
	double signal1_nav_nu;     ///< user defined dummy variable
	double signal2_nav_nu;     ///< user defined dummy variable
	double signal3_nav_nu;     ///< user defined dummy variable
	double signal4_nav_nu;     ///< user defined dummy variable
	double signal5_nav_nu;     ///< user defined dummy variable
	double signal6_nav_nu;     ///< user defined dummy variable
	double signal7_nav_nu;     ///< user defined dummy variable
	double signal8_nav_nu;     ///< user defined dummy variable
	double signal9_nav_nu;     ///< user defined dummy variable
};

/// Research Navigation Filter Data Structure
struct researchNav {
	double lat_rnav_rads;		///< [rad], geodetic latitude estimate
	double lon_rnav_rads;		///< [rad], geodetic longitude estimate
	double alt_rnav_m;		///< [m], altitude relative to WGS84 estimate
	double vn_rnav_mps;		///< [m/sec], north velocity estimate
	double ve_rnav_mps;		///< [m/sec], east velocity estimate
	double vd_rnav_mps;		///< [m/sec], down velocity estimate
	double phi_rnav_rads;		///< [rad], Euler roll angle estimate
	double the_rnav_rads;		///< [rad], Euler pitch angle estimate
	double psi_rnav_rads;		///< [rad], Euler yaw angle estimate
	double quat_rnav_nu[4];	///< Quaternions estimate
	double ab_rnav_mpsSq[3];	///< [m/sec^2], accelerometer bias estimate
	double gb_rnav_rps[3];	///< [rad/sec], rate gyro bias estimate
	double asf_rnav_mpsSq[3];	///< [m/sec^2], accelerometer scale factor estimate
	double gsf_rnav_rps[3];	///< [rad/sec], rate gyro scale factor estimate
	double Pp_rnav_rads[3];	///< [rad], covariance estimate for position
	double Pv_rnav_rads[3];	///< [rad], covariance estimate for velocity
	double Pa_rnav_rads[3];	///< [rad], covariance estimate for angles
	double Pab_rnav_rads[3];	///< [rad], covariance estimate for accelerometer bias
	double Pgb_rnav_rads[3];	///< [rad], covariance estimate for rate gyro bias
	double Pasf_rnav_rads[3];	///< [rad], covariance estimate for accelerometer scale factor
	double Pgsf_rnav_rads[3];	///< [rad], covariance estimate for rate gyro scale factor
	enum errdefs_rnav_nu;	///< NAV filter status
	double time_rnav_sec;			///< [sec], timestamp of NAV filter
	double wn_rnav_mps;			///< [m/s], estimated wind speed in the north direction
	double we_rnav_mps;			///< [m/s], estimated wind speed in the east direction
	double wd_rnav_mps;			///< [m/s], estimated wind speed in the down direction
	double signal0_rnav_nu;     ///< user defined dummy variable
	double signal1_rnav_nu;     ///< user defined dummy variable
	double signal2_rnav_nu;     ///< user defined dummy variable
	double signal3_rnav_nu;     ///< user defined dummy variable
	double signal4_rnav_nu;     ///< user defined dummy variable
	double signal5_rnav_nu;     ///< user defined dummy variable
	double signal6_rnav_nu;     ///< user defined dummy variable
	double signal7_rnav_nu;     ///< user defined dummy variable
	double signal8_rnav_nu;     ///< user defined dummy variable
	double signal9_rnav_nu;     ///< user defined dummy variable
};

/// Combined sensor data structure
struct sensordata {
	double drin_ssrdata_nu;
	double dalin_ssrdata_nu;
	double darin_ssrdata_nu;
	double dein_ssrdata_nu;
	struct imu *imuData_ssrdata_ptr; 			///< pointer to imu data structure
	struct gps *gpsData_ssrdata_ptr;			///< pointer to gps data structure
	struct gps *gpsDatal_ssrdata_ptr;			///< pointer to left gps data structure
	struct gps *gpsDatar_ssrdata_ptr;			///< pointer to right gps data structure
	struct airdata *adData_ssrdata_ptr;			///< pointer to airdata data structure
	struct surface *surfData_ssrdata_ptr;		///< pointer to surface data structure
	struct inceptor *inceptorData_ssrdata_ptr;	///< pointer to pilot inceptor data structure
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
