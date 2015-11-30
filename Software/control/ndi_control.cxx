/*
 * ndi_control.cxx
 *
 *  Created on: Oct 12, 2015
 *      Author: patipan
 */


#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "../globaldefs.h"
#include "ndi_interface.hxx"
#include "../utils/matrix.hxx"
#include "../utils/misc.hxx"
#include "../utils/transfer.hxx"
#include "../props.hxx"

// Controller variables
// gains for PID
static double p_gain[3] 	= {0.5, 0, 0};
static double q_gain[3] 	= {0.5, 0, 0};
static double r_gain[3] 	= {0.5, 0, 0};
static double phi_gain[3] 	= {0.5, 0, 0};
static double theta_gain[3] = {0.5, 0, 0};
double integrator[6] = {0, 0, 0, 0, 0, 0}; // p, q, r, phi, theta, psi integrator values
int anti_windup[6] = {1, 1, 1, 1, 1, 1};
static double curr_time, prev_time, dtime;

// yaw damper variables
//   y_yaw(z)      b[0] + b[1]*z^(-1)
//   --------  =  --------------------
//   u_yaw(z)      a[0] + a[1]*z^(-1)
static double u_yaw[2] = {0,0}; // input of filter { u(k), u(k-1) }
static double y_yaw[2] = {0,0}; // output of filter { y(k), y(k-1) }
// TODO: Determine filter coeffs for r_cmd output instead of rudder output
static double a_yaw[2] = {1.0,-0.9608}; // Filter denominator coefficients
static double b_yaw[2] = {0.065, -0.065}; // Filter numerator coefficients


// ======= Input Properties =======
// Aircraft Characteristics UltraStick25e variables
// TODO: map coeffs and values with actual aircraft data. pointer structure?

static SGPropertyNode *Cl_ac_node[6] = {NULL, NULL, NULL, NULL, NULL, NULL};	// [non-dim], Roll moment coefficients
static SGPropertyNode *Cm_ac_node[5] = {NULL, NULL, NULL, NULL, NULL};			// [non-dim], Pitch moment coefficients
static SGPropertyNode *Cn_ac_node[6] = {NULL, NULL, NULL, NULL, NULL, NULL};	// [non-dim], Yaw moment coefficients
static SGPropertyNode *wingArea_ac_m2_node = NULL;								// [m^2], aircraft wing area
static SGPropertyNode *wingSpan_ac_m_node = NULL;								// [m], aircraft wingspan
static SGPropertyNode *wingChord_ac_m_node = NULL;								// [m], aircraft wing chord

// Sensor variables
static SGPropertyNode *p_imu_rps_node = NULL; 									// [rad/sec], roll rate
static SGPropertyNode *q_imu_rps_node = NULL; 									// [rad/sec], pitch rate
static SGPropertyNode *r_imu_rps_node = NULL; 									// [rad/sec], yaw rate
static SGPropertyNode *rollAngle_nav_rads_node = NULL; 							// [rad], roll angle
static SGPropertyNode *pitchAngle_nav_rads_node = NULL; 						// [rad], pitch angle
static SGPropertyNode *aoa_ad_rads_node = NULL;									// [rad], angle of attack
static SGPropertyNode *aos_ad_rads_node = NULL;									// [rad], sideslip angle
static SGPropertyNode *ias_filt_ad_mps_node = NULL;								// [m/s], filtered airspeed
static SGPropertyNode *pd_ad_kpa_node = NULL;									// [kPa], dynamic pressure

// Control variables
static SGPropertyNode *da_r_ctrl_rads_node = NULL;								// [rad], right aileron deflection
static SGPropertyNode *de_ctrl_rads_node = NULL;								// [rad], elevator deflection
static SGPropertyNode *dr_ctrl_rads_node = NULL;								// [rad], rudder deflection
static SGPropertyNode *df_r_ctrl_rads_node = NULL;								// [rad], right flap deflection

// ======= Output Properties =======
// Control Output variables
static SGPropertyNode *phiCmd_ctrl_rads_node = NULL;							// [rad], roll command
static SGPropertyNode *theCmd_ctrl_rads_node = NULL;							// [rad], pitch command
static SGPropertyNode *pCmd_ctrl_rps_node = NULL;								// [rad/sec], roll rate command
static SGPropertyNode *qCmd_ctrl_rps_node = NULL;								// [rad/sec], pitch rate command
static SGPropertyNode *rCmd_ctrl_rps_node = NULL;								// [rad/sec], yaw rate command
static SGPropertyNode *pdotCmd_ctrl_rps2_node = NULL;							// [rad/sec^2], roll accel command
static SGPropertyNode *qdotCmd_ctrl_rps2_node = NULL;							// [rad/sec^2], pitch accel command
static SGPropertyNode *rdotCmd_ctrl_rps2_node = NULL;							// [rad/sec^2], yaw accel command
static SGPropertyNode *pdotAlloc_ctrl_rps2_node = NULL;							// [rad/sec^2], roll accel to control allocation
static SGPropertyNode *qdotAlloc_ctrl_rps2_node = NULL;							// [rad/sec^2], pitch accel to control allocation
static SGPropertyNode *rdotAlloc_ctrl_rps2_node = NULL;							// [rad/sec^2], yaw accel to control allocation


// ======= Input Local Variables =======
// Aircraft characteristics
//Cl_array[6] = {Cl_beta, Cl_p, Cl_r, Cl_ail, Cl_rud, Cl_flap};
//Cm_array[5] = {Cm_alpha, Cm_q, Cm_elev, Cm_flap, Cm_ail};
//Cn_array[6] = {Cn_beta, Cn_p, Cn_r, Cn_ail, Cn_rud, Cn_flap};
static double Cl_array[6] = {-0.0545, -0.4496, 0.1086, -0.1646, 0.0115, -0.0692};
static double Cm_array[5] = {-0.7230, -13.5664, -0.8488, 0.0467, 0.0467};
static double Cn_array[6] = {0.0723, 0.1180, -0.1833, 0.0574, -0.1811, 0.0214};
static double S = 0.3097; // Wing area [m^2]
static double b = 1.27; // wing span [m]
static double c = 0.25; // wing chord [m]

// Sensor variables
static double p_imu_rps = 0.0;							// [rad/sec], roll rate
static double q_imu_rps = 0.0;							// [rad/sec], pitch rate
static double r_imu_rps = 0.0;							// [rad/sec], yaw rate
static double rollAngle_nav_rads = 0.0;					// [rad], roll angle
static double pitchAngle_nav_rads = 0.0;				// [rad], pitch angle
static double aoa_ad_rads = 0.0;						// [rad], angle of attack
static double aos_ad_rads = 0.0;						// [rad], sideslip angle
static double ias_filt_ad_mps = 0.0;					// [m/s], filtered airspeed
static double pd_ad_kpa = 0.0;							// [kPa], dynamic pressure

// Control variables
static double da_r_ctrl_rads = 0.0;						// [rad], right aileron deflection
static double de_ctrl_rads = 0.0;						// [rad], elevator deflection
static double dr_ctrl_rads = 0.0;						// [rad], rudder deflection
static double df_r_ctrl_rads = 0.0;						// [rad], right flap deflection

// ======= Output Local Variables =======
static double phiCmd_ctrl_rads = 0.0;					// [rad], roll command
static double theCmd_ctrl_rads = 0.0;					// [rad], pitch command
static double pCmd_ctrl_rps = 0.0;						// [rad/sec], roll rate command
static double qCmd_ctrl_rps = 0.0;						// [rad/sec], pitch rate command
static double rCmd_ctrl_rps = 0.0;						// [rad/sec], yaw rate command
static double pdotCmd_ctrl_rps2 = 0.0;					// [rad/sec^2], roll accel command
static double qdotCmd_ctrl_rps2 = 0.0;					// [rad/sec^2], pitch accel command
static double rdotCmd_ctrl_rps2 = 0.0;					// [rad/sec^2], yaw accel command
static double pdotAlloc_ctrl_rps2 = 0.0;				// [rad/sec^2], roll accel to control allocation
static double qdotAlloc_ctrl_rps2 = 0.0;				// [rad/sec^2], pitch accel to control allocation
static double rdotAlloc_ctrl_rps2 = 0.0;				// [rad/sec^2], yaw accel to control

// ======= Controller Variables =======
// ref model regression variables
static double p_RegX1, p_RegX2, p_RegY1, p_RegY2; // roll rate ref model vars
static double p_num[3] = {0.0, 0.0, 0.5}; // TF numerator coefficients
static double p_den[3] = {0.0, 1, 0.5}; // TF denominator coefficients

// matrices for dynamic inversion
static MATRIX I_matrix, I_inv, pqr, pqrdot_cmd, Moments, pqrdot_inv;



// ======= Define local functions =======
static void dynamic_inverse();
static void level4_control(double dtime);
static void level3_control(double dtime);
static double pid(double err, double integrator, double derivative, double gain[3], int anti_windup);


void init_control()
{
	prev_time = get_Time();
	printf("\tCreating matrices...\n");
	I_matrix = mat_creat(3, 3, ONES_MATRIX);
	// TODO: fill in I_matrix, get from AC data
	I_matrix[0][0] = 0.07151;	I_matrix[0][1] = 0.0;		I_matrix[0][2] = 0.07151;
	I_matrix[1][0] = 0.0;		I_matrix[1][1] = 0.08636;	I_matrix[1][2] = 0.0;
	I_matrix[2][0] = 0.014;		I_matrix[2][1] = 0.0;		I_matrix[2][2] = 0.15364;

	// ======= Initialize dynamic inversion matrices =======
	I_inv 		= mat_creat(3, 3, ZERO_MATRIX);
	I_inv 		= mat_inv(I_matrix, I_inv); 	// inverse inertia
	pqr 		= mat_creat(3, 1, ZERO_MATRIX); // p, q, r matrix
	pqrdot_cmd 	= mat_creat(3, 1, ZERO_MATRIX); // pdot, qdot, rdot commands
	Moments 	= mat_creat(3, 1, ZERO_MATRIX); // aircraft moments
	pqrdot_inv 	= mat_creat(3, 1, ZERO_MATRIX); // pdot, qdot, rdot allocation command

	// ------- init regession variables for reference models -------
	p_RegX1 = p_RegX2 = p_RegY1 = p_RegY2 = 0; // roll rate ref model vars

	// ======= Variables for concatenating path =======
	char imu[] = "/imu";
	char nav[] = "/nav";
	char ad[] = "/ad";
	char ctrl[] = "/ctrl";

	printf("\tInitializing property nodes...\n");
	// ======= Property node initialization =======
	// Aircraft variables
	// TODO: determine how to get aircraft characteristics

	// Sensor variables

	p_imu_rps_node = fgGetNode(imu, "/p_imu_rps", 0, true); 					// [rad/sec], roll rate
	q_imu_rps_node = fgGetNode(imu, "/q_imu_rps", 0, true); 					// [rad/sec], pitch rate
	r_imu_rps_node = fgGetNode(imu, "/r_imu_rps", 0, true); 					// [rad/sec], yaw rate
	rollAngle_nav_rads_node = fgGetNode(nav, "/rollAngle_nav_rads", 0, true); 	// [rad], roll angle
	pitchAngle_nav_rads_node = fgGetNode(nav, "/pitchAngle_nav_rads", 0, true); // [rad], pitch angle
	aoa_ad_rads_node = fgGetNode(ad, "/aoa_ad_rads", 0, true); 					// [rad], angle of attack
	aos_ad_rads_node = fgGetNode(ad, "/aos_ad_rads", 0, true); 					// [rad], sideslip angle
	ias_filt_ad_mps_node = fgGetNode(ad, "/ias_filt_ad_mps", 0, true); 			// [m/s], filtered airspeed
	pd_ad_kpa_node = fgGetNode(ad, "/pd_ad_kpa", 0, true); 						// [kPa], dynamic pressure

	// Control variables
	da_r_ctrl_rads_node = fgGetNode(ctrl, "/da_r_ctrl_rads", 0, true);			// [rad], right aileron deflection
	de_ctrl_rads_node = fgGetNode(ctrl, "/de_ctrl_rads", 0, true);				// [rad], elevator deflection
	dr_ctrl_rads_node = fgGetNode(ctrl, "/dr_ctrl_rads", 0, true);				// [rad], rudder deflection
	df_r_ctrl_rads_node = fgGetNode(ctrl, "/df_r_ctrl_rads", 0, true);			// [rad], right flap deflection
	phiCmd_ctrl_rads_node = fgGetNode(ctrl, "/phiCmd_ctrl_rads", 0, true);		// [rad], roll command
	theCmd_ctrl_rads_node = fgGetNode(ctrl, "/theCmd_ctrl_rads", 0, true);		// [rad], pitch command
	pCmd_ctrl_rps_node = fgGetNode(ctrl, "/pCmd_ctrl_rps", 0, true);			// [rad/sec], roll rate command
	qCmd_ctrl_rps_node = fgGetNode(ctrl, "/qCmd_ctrl_rps", 0, true);			// [rad/sec], pitch rate command
	rCmd_ctrl_rps_node = fgGetNode(ctrl, "/rCmd_ctrl_rps", 0, true);			// [rad/sec], yaw rate command
	pdotCmd_ctrl_rps2_node = fgGetNode(ctrl, "/pdptCmd_ctrl_rps2", 0, true);	// [rad/sec^2], roll accel command
	qdotCmd_ctrl_rps2_node = fgGetNode(ctrl, "/qdptCmd_ctrl_rps2", 0, true);	// [rad/sec^2], pitch accel command
	rdotCmd_ctrl_rps2_node = fgGetNode(ctrl, "/rdptCmd_ctrl_rps2", 0, true);	// [rad/sec^2], yaw accel command
	pdotAlloc_ctrl_rps2_node = fgGetNode(ctrl, "/pdotAlloc_ctrl_rps2", 0, true);// [rad/sec^2], roll accel to control allocation
	qdotAlloc_ctrl_rps2_node = fgGetNode(ctrl, "/qdotAlloc_ctrl_rps2", 0, true);// [rad/sec^2], pitch accel to control allocation
	rdotAlloc_ctrl_rps2_node = fgGetNode(ctrl, "/rdotAlloc_ctrl_rps2", 0, true);// [rad/sec^2], yaw accel to control

	printf("\tReading values...\n");
	// ======= Read values from property tree =======
	// Input local variables
	// Sensor variables
	p_imu_rps = p_imu_rps_node->getDoubleValue(); 						// [rad/sec], roll rate
	q_imu_rps = q_imu_rps_node->getDoubleValue(); 						// [rad/sec], pitch rate
	r_imu_rps = r_imu_rps_node->getDoubleValue(); 						// [rad/sec], yaw rate
	rollAngle_nav_rads = rollAngle_nav_rads_node->getDoubleValue(); 	// [rad], roll angle
	pitchAngle_nav_rads = pitchAngle_nav_rads_node->getDoubleValue(); 	// [rad], pitch angle
	aoa_ad_rads = aoa_ad_rads_node->getDoubleValue(); 					// [rad], angle of attack
	aos_ad_rads = aos_ad_rads_node->getDoubleValue();					// [rad], sideslip angle
	ias_filt_ad_mps = ias_filt_ad_mps_node->getDoubleValue(); 			// [m/s], filtered airspeed
	pd_ad_kpa = pd_ad_kpa_node->getDoubleValue();						// [kPa], dynamic pressure

	printf("\tSensors read...Reading control vars...\n");
	// Control variables
	da_r_ctrl_rads = da_r_ctrl_rads_node->getDoubleValue();				// [rad], right aileron deflection
	de_ctrl_rads = de_ctrl_rads_node->getDoubleValue();					// [rad], elevator deflection
	dr_ctrl_rads = dr_ctrl_rads_node->getDoubleValue();					// [rad], rudder deflection
	df_r_ctrl_rads = df_r_ctrl_rads_node->getDoubleValue();				// [rad], right flap deflection

	// ======= Initialize local output variables =======
	// Control variables
	phiCmd_ctrl_rads = 0.0;						// [rad], roll command
	theCmd_ctrl_rads = 0.0;						// [rad], pitch command
	pCmd_ctrl_rps = 0.0;						// [rad/sec], roll rate command
	qCmd_ctrl_rps = 0.0;						// [rad/sec], pitch rate command
	rCmd_ctrl_rps = 0.0;						// [rad/sec], yaw rate command
	pdotCmd_ctrl_rps2 = 0.0;					// [rad/sec^2], roll accel command
	qdotCmd_ctrl_rps2 = 0.0;					// [rad/sec^2], pitch accel command
	rdotCmd_ctrl_rps2 = 0.0;					// [rad/sec^2], yaw accel command
	pdotAlloc_ctrl_rps2 = 0.0;					// [rad/sec^2], roll accel to control allocation
	qdotAlloc_ctrl_rps2 = 0.0;					// [rad/sec^2], pitch accel to control allocation
	rdotAlloc_ctrl_rps2 = 0.0;					// [rad/sec^2], yaw accel to control

	printf("\tinit_control() finished\n\n");
} // end init_control()

void get_control()
{
	curr_time = get_Time();
	dtime = curr_time - prev_time; // [sec]
	prev_time = curr_time; // Might set dtime as a constant global variable
	//level1_control();
	//level2_control();

	// Run outerloop control
	level3_control(dtime);

	// Run innerloop control
	level4_control(dtime);

	// Run dynamic inversion
	dynamic_inverse();
}

static void dynamic_inverse()
{
	// Take pdot, qdot, rdot commands and output angular acceleration commands

	// Get angular accel commands from level4 controller
	pdotCmd_ctrl_rps2 = pdotCmd_ctrl_rps2_node->getDoubleValue();
	qdotCmd_ctrl_rps2 = qdotCmd_ctrl_rps2_node->getDoubleValue();
	rdotCmd_ctrl_rps2 = rdotCmd_ctrl_rps2_node->getDoubleValue();

	// measured p, q, r
	pqr[0][0] = p_imu_rps;
	pqr[1][0] = q_imu_rps;
	pqr[2][0] = r_imu_rps;

	// surface deflections, use measured pot values if available
	// created a fake beta and alpha if not measured. use theta, etc. .... how?
	// TODO: determine alpha and beta for single hole probe OR use theta instead
	double Cl = Cl_array[0]*aos_ad_rads
				+ Cl_array[1]*p_imu_rps*b/(2*ias_filt_ad_mps)
				+ Cl_array[2]*r_imu_rps*b/(2*ias_filt_ad_mps)
				+ Cl_array[3]*da_r_ctrl_rads
				+ Cl_array[4]*dr_ctrl_rads
				+ Cl_array[5]*df_r_ctrl_rads;
	double Cm = Cm_array[0]*aoa_ad_rads
				+ Cm_array[1]*q_imu_rps*c/2/ias_filt_ad_mps
				+ Cm_array[2]*de_ctrl_rads
				+ Cm_array[3]*df_r_ctrl_rads
				+ Cm_array[4]*da_r_ctrl_rads;
	double Cn = Cn_array[0]*aos_ad_rads
				+ Cn_array[1]*p_imu_rps*b/(2*ias_filt_ad_mps)
				+ Cn_array[2]*r_imu_rps*b/(2*ias_filt_ad_mps)
				+ Cn_array[3]*da_r_ctrl_rads
				+ Cn_array[4]*dr_ctrl_rads
				+ Cn_array[5]*df_r_ctrl_rads;

	double L = pd_ad_kpa*S*b*Cl;
	double M = pd_ad_kpa*S*c*Cm;
	double N = pd_ad_kpa*S*b*Cn;

	printf("Roll moment (L): %f [Nm]\n", L);
	printf("Pitch moment (M): %f[Nm]\n", M);
	printf("Yaw moment (N): %f [Nm]\n", N);

	// Aircraft moments matrix
	Moments[0][0] = L;
	Moments[1][0] = M;
	Moments[2][0] = N;

	// Compute the inversion values: wdot_inv = I^(-1)*(Moments - w x I*w)
	MATRIX I_w, cross, temp;
	I_w 		= mat_creat(3,1,ZERO_MATRIX);
	cross 		= mat_creat(3,1,ZERO_MATRIX);
	temp 		= mat_creat(3,1,ZERO_MATRIX);
	I_w 		= mat_mul(I_matrix, pqr, I_w);
	cross 		= mat_cross(pqr, I_w, cross);
	temp 		= mat_sub(Moments, cross, temp);
	pqrdot_inv 	= mat_mul(I_inv, temp, pqrdot_inv);

	// Compute control allocation
	pdotAlloc_ctrl_rps2 = pdotCmd_ctrl_rps2 - pqrdot_inv[0][0];
	qdotAlloc_ctrl_rps2 = qdotCmd_ctrl_rps2 - pqrdot_inv[1][0];
	rdotAlloc_ctrl_rps2 = rdotCmd_ctrl_rps2 - pqrdot_inv[2][0];

	printf("pdotAlloc_ctrl: %f [rad/s^2]\n", pdotAlloc_ctrl_rps2);
	// Set values to property tree
	pdotAlloc_ctrl_rps2_node->setDoubleValue(pdotAlloc_ctrl_rps2);
	qdotAlloc_ctrl_rps2_node->setDoubleValue(qdotAlloc_ctrl_rps2);
	rdotAlloc_ctrl_rps2_node->setDoubleValue(rdotAlloc_ctrl_rps2);
	printf("pdotAlloc_ctrl_node: %f [rad/s^2]\n", pdotAlloc_ctrl_rps2_node->getDoubleValue());

	// TODO: Free matrices
}

static void level4_control(double dtime)
{
	// output pdot, qdot, rdot commands from p,q,r with response shape filter

	// Local function variables
	double p_ref, q_ref, r_ref, p_err, q_err, r_err;

	// Get angular rates from level3 controller
	pCmd_ctrl_rps = pCmd_ctrl_rps_node->getDoubleValue();
	qCmd_ctrl_rps = qCmd_ctrl_rps_node->getDoubleValue();
	rCmd_ctrl_rps = rCmd_ctrl_rps_node->getDoubleValue();

	// TODO: Reference model transfer function
	p_ref = tf(pCmd_ctrl_rps, p_num, p_den, &p_RegX1, &p_RegX2, &p_RegY1, &p_RegY2, 0.02);
	//q_ref = qCmd_ctrl_rps;
	//r_ref = rCmd_ctrl_rps * TF;
	//p_ref = p_cmd;
	q_ref = qCmd_ctrl_rps;
	r_ref = rCmd_ctrl_rps;
	p_err = p_ref - p_imu_rps;
	q_err = q_ref - q_imu_rps;
	r_err = r_ref - r_imu_rps;

	// ------- p controller -------
	integrator[0] += p_err * dtime; // Add anti-windup
	if      (da_r_ctrl_rads >= AILERON_AUTH_MAX && p_err > 0) {anti_windup[0] = 1;}
	else if (da_r_ctrl_rads >= AILERON_AUTH_MAX && p_err < 0) {anti_windup[0] = 0;}  //stop integrating
	else if (da_r_ctrl_rads <= -AILERON_AUTH_MAX && p_err > 0) {anti_windup[0] = 0;}  //stop integrating
	else if (da_r_ctrl_rads <= -AILERON_AUTH_MAX && p_err < 0) {anti_windup[0] = 1;}
	else {anti_windup[0] = 1;}
	pdotCmd_ctrl_rps2 = pid(p_err, integrator[0], 0, p_gain, anti_windup[0]); // ang accel not measured, PI control

	// ------- q controller -------
	integrator[1] += q_err * dtime; // Add anti-windup
	if      (de_ctrl_rads >= ELEVATOR_AUTH_MAX && q_err > 0) {anti_windup[1] = 1;}
	else if (de_ctrl_rads >= ELEVATOR_AUTH_MAX && q_err < 0) {anti_windup[1] = 0;} //stop integrating
	else if (de_ctrl_rads <= -ELEVATOR_AUTH_MAX && q_err > 0) {anti_windup[1] = 0;} //stop integrating
	else if (de_ctrl_rads <= -ELEVATOR_AUTH_MAX && q_err < 0) {anti_windup[1] = 1;}
	else {anti_windup[1] = 1;}
	qdotCmd_ctrl_rps2 = pid(q_err, integrator[1], 0, q_gain, anti_windup[1]); // ang accel not measured, PI control

	// ------- r controller -------
	integrator[2] += r_err * dtime; // Add anti-windup
	if      (dr_ctrl_rads >= RUDDER_AUTH_MAX && r_err > 0) {anti_windup[2] = 1;}
	else if (dr_ctrl_rads >= RUDDER_AUTH_MAX && r_err < 0) {anti_windup[2] = 0;} //stop integrating
	else if (dr_ctrl_rads <= RUDDER_AUTH_MAX && r_err > 0) {anti_windup[2] = 0;} //stop integrating
	else if (dr_ctrl_rads <= RUDDER_AUTH_MAX && r_err < 0) {anti_windup[2] = 1;}
	else {anti_windup[2] = 1;}
	rdotCmd_ctrl_rps2 = pid(r_err, integrator[2], 0, r_gain, anti_windup[2]); // ang accel not measured, PI control

	// Set values to property tree
	pdotCmd_ctrl_rps2_node->setDoubleValue(pdotCmd_ctrl_rps2);
	qdotCmd_ctrl_rps2_node->setDoubleValue(qdotCmd_ctrl_rps2);
	rdotCmd_ctrl_rps2_node->setDoubleValue(rdotCmd_ctrl_rps2);

	printf("level4 pdotcmd: %f dps2\n", pdotCmd_ctrl_rps2*R2D);
}


static void level3_control(double dtime)
{
	// Take phi, theta commands, and a yaw damper and output p, q, r commands

	// Get values from property tree
	phiCmd_ctrl_rads = phiCmd_ctrl_rads_node->getDoubleValue();					// [rad], roll command
	theCmd_ctrl_rads = theCmd_ctrl_rads_node->getDoubleValue();					// [rad], pitch command

	// Local function variables
	double phi_ref, theta_ref, phi_err, theta_err;

	// TODO: Reference model transfer function
	//phi_ref = phiCmd_ctrl_rads * TF;
	//theta_ref = theta_cmd * TF;
	//psi_ref = psi_cmd * TF;
	phi_ref 	= phiCmd_ctrl_rads;
	theta_ref 	= theCmd_ctrl_rads;

	phi_err 	= phi_ref - rollAngle_nav_rads;
	theta_err 	= theta_ref - pitchAngle_nav_rads;

	// ------- Roll Controller -------
	integrator[3] += phi_err*dtime;
	pCmd_ctrl_rps = pid(phi_err, integrator[3], p_imu_rps, phi_gain, 1);

	// ------- Pitch Controller -------
	integrator[4] += theta_err*dtime;
	qCmd_ctrl_rps = pid(theta_err, integrator[4], q_imu_rps, theta_gain, 1);

	// ------- Yaw Damper -------
	// Discrete time filter equation at time step k
	// y_yaw(k) = b0*u(k) + b1*u(k-1) - a1*y(k-1)
	u_yaw[1] = r_imu_rps;  //current time step input (r)

	// Filter:
	// y_yaw(k) = b0*u(k) +	b1*u(k-1) - a1*y(k-1)
	y_yaw[1] = b_yaw[0]*u_yaw[1] + b_yaw[1]*u_yaw[0] - a_yaw[1]*y_yaw[1];

	// Update past time step input/output for next frame
	u_yaw[0] = u_yaw[1];
	y_yaw[0] = y_yaw[1];

	// Output yawrate command
	rCmd_ctrl_rps = y_yaw[1];

	// Set values to property tree
	pCmd_ctrl_rps_node->setDoubleValue(pCmd_ctrl_rps);
	qCmd_ctrl_rps_node->setDoubleValue(qCmd_ctrl_rps);
	rCmd_ctrl_rps_node->setDoubleValue(rCmd_ctrl_rps);

	printf("level3 pcmd: %f dps\n", pCmd_ctrl_rps*R2D);
}

/*
static void level2_control(struct control *controlData_ptr, struct sensordata *sensorData_ptr, struct nav *navData_ptr)
{
	// Take psi, gamma, velocity commands and output phi, alpha, Vdot commands

}
*/
/*
static void level1_control(struct control *controlData_ptr, struct sensordata *sensorData_ptr, struct nav *navData_ptr)
{
	// Take lat, long, h commands and output psi, gamma commands
}
*/

static double pid(double err, double integrator, double derivative, double gain[3], int anti_windup)
{
	double output;
	output = gain[0]*err + gain[1]*integrator*anti_windup - gain[2]*derivative;
	return output;
}

