
#ifndef GLOBALDEFS_HXX_
#define GLOBALDEFS_HXX_

#define D2R 0.017453292519943
#define R2D 57.295779513082323
#define	g	9.814				///< [m/sec^2] gravity */

/// GPS Data Structure
struct gps {
	double lat_rad;				///< [rad], Geodetic latitude
	double lon_rad;				///< [rad], Geodetic longitude
	double alt_m;				///< [m], altitude relative to MSL
	double ve_mps;				///< [m/sec], East velocity
	double vn_mps;				///< [m/sec], North velocity
	double vd_mps;				///< [m/sec], Down velocity
	double hAcc_m;				///< [m], horizontal accuracy
	double vAcc_m;				///< [m], vertical accuracy
	double sAcc_mps;			///< [m/sec], speed accuracy
	unsigned short valid;		///< flag indicating whether the solution is valid, 1 = valid
	unsigned short newData;		///< [bool], flag set when GPS data has been updated
	unsigned short satVisible; 	///< Number satellites used in the position solution
};

/// IMU Data Structure
struct imu {
	double p_rads;		///< [rad/sec], body X axis angular rate (roll)
	double q_rads;		///< [rad/sec], body Y axis angular rate (pitch)
	double r_rads;		///< [rad/sec], body Z axis angular rate (yaw)
	double ax_mss;		///< [m/sec^2], body X axis acceleration
	double ay_mss;		///< [m/sec^2], body Y axis acceleration
	double az_mss;		///< [m/sec^2], body Z axis acceleration
	double time_sec; 	///< [sec], timestamp of IMU data
};

/// Navigation Filter Data Structure
struct nav {
	double lat_rad;			///< [rad], geodetic latitude estimate
	double lon_rad;			///< [rad], geodetic longitude estimate
	double alt_m;			///< [m], altitude relative to WGS84 estimate
	double vn_mps;			///< [m/sec], north velocity estimate
	double ve_mps;			///< [m/sec], east velocity estimate
	double vd_mps;			///< [m/sec], down velocity estimate
	double phi_rad;			///< [rad], Euler roll angle estimate
	double theta_rad;		///< [rad], Euler pitch angle estimate
	double psi_rad;			///< [rad], Euler yaw angle estimate
	double quat[4];			///< Quaternions estimate
	double ab[3];			///< [m/sec^2], accelerometer bias estimate
	double gb[3];			///< [rad/sec], rate gyro bias estimate
	double asf[3];			///< [m/sec^2], accelerometer scale factor estimate
	double gsf[3];			///< [rad/sec], rate gyro scale factor estimate
	double Pp[3];			///< [rad], covariance estimate for position
	double Pv[3];			///< [rad], covariance estimate for velocity
	double Pa[3];			///< [rad], covariance estimate for angles
	double Pab[3];			///< [rad], covariance estimate for accelerometer bias
	double Pgb[3];			///< [rad], covariance estimate for rate gyro bias
	double Pasf[3];			///< [rad], covariance estimate for accelerometer scale factor
	double Pgsf[3];			///< [rad], covariance estimate for rate gyro scale factor
	unsigned short status;	///< [bool], whether the filter has been initialized
};

#endif

