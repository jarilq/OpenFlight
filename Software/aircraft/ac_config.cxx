/*
 * aircraft_config.cxx
 *
 *  Created on: Feb 25, 2016
 *      Author: patipan
 */

#include "ac_config.hxx"

#include "hdf5.h"
#include <stdio.h>
#include <stdlib.h>
#include "../props.hxx"
#include "ac_config.hxx"

// ======= Local Function =======
void do_dset(char *, hid_t, void *mem);
void calcCapeChecksum(unsigned char* CK);
cape capePacket;

// ======= Output Properties =======
// Aircraft Characteristics
static SGPropertyNode *Cl_beta_ac_node = NULL;								// [non-dim], Roll moment coefficient (beta)
static SGPropertyNode *Cl_p_ac_node = NULL;									// [non-dim], Roll moment coefficient (p)
static SGPropertyNode *Cl_r_ac_node = NULL;									// [non-dim], Roll moment coefficient (r)
static SGPropertyNode *Cl_surf_ac_node[20] = {NULL, NULL, NULL, NULL, NULL, // [non-dim], Roll moment coefficients (surfaces)
											  NULL, NULL, NULL, NULL, NULL,
											  NULL, NULL, NULL, NULL, NULL,
											  NULL, NULL, NULL, NULL, NULL};
static SGPropertyNode *Cm_alpha_ac_node = NULL;								// [non-dim], Pitch moment coefficient (alpha)
static SGPropertyNode *Cm_q_ac_node = NULL;									// [non-dim], Pitch moment coefficient (q)
static SGPropertyNode *Cm_zero_ac_node = NULL;								// [non-dim], Pitch moment coefficient (zero)
static SGPropertyNode *Cm_surf_ac_node[20] = {NULL, NULL, NULL, NULL, NULL, // [non-dim], Pitch moment coefficients (surfaces)
											  NULL, NULL, NULL, NULL, NULL,
											  NULL, NULL, NULL, NULL, NULL,
											  NULL, NULL, NULL, NULL, NULL};
static SGPropertyNode *Cn_beta_ac_node = NULL;								// [non-dim], Yaw moment coefficient (beta)
static SGPropertyNode *Cn_p_ac_node = NULL;									// [non-dim], Yaw moment coefficient (p)
static SGPropertyNode *Cn_r_ac_node = NULL;									// [non-dim], Yaw moment coefficient (r)
static SGPropertyNode *Cn_surf_ac_node[20] = {NULL, NULL, NULL, NULL, NULL, // [non-dim], Yaw moment coefficients (surfaces)
											  NULL, NULL, NULL, NULL, NULL,
											  NULL, NULL, NULL, NULL, NULL,
											  NULL, NULL, NULL, NULL, NULL};
static SGPropertyNode *CL_alpha_node = NULL;								// [non-dim], Lift (alpha) coefficient
static SGPropertyNode *wingArea_ac_m2_node = NULL;							// [m^2], aircraft wing area
static SGPropertyNode *wingSpan_ac_m_node = NULL;							// [m], aircraft wingspan
static SGPropertyNode *wingChord_ac_m_node = NULL;							// [m], aircraft wing chord
static SGPropertyNode *Ixx_ac_kgm2_node = NULL;								// [kg*m^2] moment of inertia
static SGPropertyNode *Ixz_ac_kgm2_node = NULL;								// [kg*m^2] moment of inertia
static SGPropertyNode *Iyy_ac_kgm2_node = NULL;								// [kg*m^2] moment of inertia
static SGPropertyNode *Izz_ac_kgm2_node = NULL;								// [kg*m^2] moment of inertia
static SGPropertyNode *mass_ac_kg_node = NULL;								// [kg] aircraft's mass
static SGPropertyNode *rProp_ac_m_node[3] = {NULL, NULL, NULL};				// [m] location of prop system
static SGPropertyNode *anglesProp_ac_rad_node[3]= {NULL, NULL, NULL};		// [rad] angle of prop system
static SGPropertyNode *rCG_ac_m_node[3] = {NULL, NULL, NULL};				// [m] location of CG in body (x,y,z)
static SGPropertyNode *prop_radius_ac_m_node = NULL;						// [m] propeller radius
static SGPropertyNode *prop_CT_ac_node[5] = {NULL, NULL, NULL, NULL, NULL};	// [non-dim], Thrust coefficients
static SGPropertyNode *PWMOut_ac_node[20] = {NULL, NULL, NULL, NULL, NULL, 	// [non-dim], PWM Output mapping
											 NULL, NULL, NULL, NULL, NULL,
											 NULL, NULL, NULL, NULL, NULL,
											 NULL, NULL, NULL, NULL, NULL};
static SGPropertyNode *Nz_lim_ac_g_node = NULL;								// [G] aircraft load limit
static SGPropertyNode *alpha_lim_ac_rad_node = NULL;						// [rad] angle of attack limit
static SGPropertyNode *beta_lim_ac_rad_node = NULL;							// [rad] sideslip limit
static SGPropertyNode *p_lim_ac_node = NULL;								// [non-dim] roll rate limit (p*b)/(2*V)
static SGPropertyNode *q_lim_ac_node = NULL;								// [non-dim] pitch rate limit (q*c)/(2*V)
static SGPropertyNode *r_lim_ac_node = NULL;								// [non-dim] yaw rate limit (r*b)/(2*V)
static SGPropertyNode *phi_lim_ac_rad_node = NULL;							// [rad] bank angle limit

// ======= Output Local Variables =======
static double Cl_beta_ac;													// [non-dim], Roll moment coefficient (beta)
static double Cl_p_ac;														// [non-dim], Roll moment coefficient (p)
static double Cl_r_ac;														// [non-dim], Roll moment coefficient (r)
static double Cl_surf_ac[20];												// [non-dim], Roll moment coefficients (surfaces)
static double Cm_alpha_ac;													// [non-dim], Pitch moment coefficient (alpha)
static double Cm_q_ac;														// [non-dim], Pitch moment coefficient (q)
static double Cm_zero_ac;													// [non-dim], Pitch moment coefficient (zero)
static double Cm_surf_ac[20];												// [non-dim], Pitch moment coefficients (surfaces)
static double Cn_beta_ac;													// [non-dim], Yaw moment coefficient (beta)
static double Cn_p_ac;														// [non-dim], Yaw moment coefficient (p)
static double Cn_r_ac;														// [non-dim], Yaw moment coefficient (r)
static double Cn_surf_ac[20]; 												// [non-dim], Yaw moment coefficients (surfaces)
static double CL_alpha;														// [non-dim], Lift (alpha) coefficient
static double wingArea_ac_m2;												// [m^2], aircraft wing area
static double wingSpan_ac_m;												// [m], aircraft wingspan
static double wingChord_ac_m;												// [m], aircraft wing chord
static double Ixx_ac_kgm2;													// [kg*m^2] moment of inertia
static double Ixz_ac_kgm2;													// [kg*m^2] moment of inertia
static double Iyy_ac_kgm2;													// [kg*m^2] moment of inertia
static double Izz_ac_kgm2;													// [kg*m^2] moment of inertia
static double mass_ac_kg;													// [kg] aircraft's mass
static double rProp_ac_m[3];												// [m] location of prop system
static double anglesProp_ac_rad[3];											// [rad] angle of prop system
static double rCG_ac_m[3];													// [m] location of CG in body (x,y,z)
static double prop_radius_ac_m;												// [m] propeller radius
static double prop_CT_ac[5];												// [non-dim], Thrust coefficients
static int PWMOut_ac[20];													// [non-dim], PWM Output mapping
static double Nz_lim_ac_g;													// [G] aircraft load limit
static double alpha_lim_ac_rad;												// [rad] angle of attack limit
static double beta_lim_ac_rad;												// [rad] sideslip limit
static double p_lim_ac;														// [non-dim] roll rate limit (p*b)/(2*V)
static double q_lim_ac;														// [non-dim] pitch rate limit (q*c)/(2*V)
static double r_lim_ac;														// [non-dim] yaw rate limit (r*b)/(2*V)
static double phi_lim_ac_rad;												// [rad] bank angle limit

void get_config(char * FILE)
{
	// ======= Variables for concatenating path =======
	char ac[] = "/ac";

	// ======= Property node initialization =======
	// Aircraft variables
	Cl_beta_ac_node = fgGetNode(ac, (char *)"/Cl_beta_ac", 0, true);		// [non-dim], Cl_beta
	Cl_p_ac_node = fgGetNode(ac, (char *)"/Cl_p_ac", 0, true);				// [non-dim], Cl_p
	Cl_r_ac_node = fgGetNode(ac, (char *)"/Cl_r_ac", 0, true);				// [non-dim], Cl_r
	for(int i =0; i<20; i++)
	{
		Cl_surf_ac_node[i] = fgGetNode(ac, (char *)"/Cl_surf_ac", i, true);	// [non-dim], Cl_surfaces
		Cm_surf_ac_node[i] = fgGetNode(ac, (char *)"/Cm_surf_ac", i, true);	// [non-dim], Cm_surfaces
		Cn_surf_ac_node[i] = fgGetNode(ac, (char *)"/Cn_surf_ac", i, true);	// [non-dim], Cn_surfaces
		PWMOut_ac_node[i] = fgGetNode(ac, (char *)"/PWMOut_ac", i, true);	// [non-dim], PWM Output mapping
	}
	Cm_alpha_ac_node = fgGetNode(ac, (char *)"/Cm_alpha_ac", 0, true);		// [non-dim], Cm_alpha
	Cm_q_ac_node = fgGetNode(ac, (char *)"/Cm_q_ac", 0, true);				// [non-dim], Cm_q
	Cm_zero_ac_node = fgGetNode(ac, (char *)"/Cm_zero_ac", 0, true);		// [non-dim], Cm_zero
	Cn_beta_ac_node = fgGetNode(ac, (char *)"/Cn_beta_ac", 0, true);		// [non-dim], Cn_beta
	Cn_p_ac_node = fgGetNode(ac, (char *)"/Cn_p_ac", 0, true);				// [non-dim], Cn_p
	Cn_r_ac_node = fgGetNode(ac, (char *)"/Cn_r_ac", 0, true);				// [non-dim], Cn_r
	CL_alpha_node = fgGetNode(ac, (char *)"/CL_alpha", 0, true);			// [non-dim], Lift (alpha) coefficient
	wingArea_ac_m2_node = fgGetNode(ac, (char *)"/wingArea_ac_m2", 0, true);// [m^2], aircraft wing area
	wingSpan_ac_m_node = fgGetNode(ac, (char *)"/wingSpan_ac_m", 0, true);	// [m], aircraft wingspan
	wingChord_ac_m_node = fgGetNode(ac, (char *)"/wingChord_ac_m", 0, true);// [m], aircraft wing chord
	Ixx_ac_kgm2_node = fgGetNode(ac, (char *)"/Ixx_ac_kgm2", 0, true);		// [kg*m^2] moment of inertia
	Ixz_ac_kgm2_node = fgGetNode(ac, (char *)"/Ixz_ac_kgm2", 0, true);		// [kg*m^2] moment of inertia
	Iyy_ac_kgm2_node = fgGetNode(ac, (char *)"/Iyy_ac_kgm2", 0, true);		// [kg*m^2] moment of inertia
	Izz_ac_kgm2_node = fgGetNode(ac, (char *)"/Izz_ac_kgm2", 0, true);		// [kg*m^2] moment of inertia
	mass_ac_kg_node = fgGetNode(ac, (char *)"/mass_ac_kg", 0, true);		// [kg] aircraft's mass
	rProp_ac_m_node[0] = fgGetNode(ac, (char *)"/rProp_ac_m", 0, true);		// [m] x location of prop system
	rProp_ac_m_node[1] = fgGetNode(ac, (char *)"/rProp_ac_m", 1, true);		// [m] y location of prop system
	rProp_ac_m_node[2] = fgGetNode(ac, (char *)"/rProp_ac_m", 2, true);		// [m] z location of prop system
	anglesProp_ac_rad_node[0]= fgGetNode(ac, (char *)"/anglesProp_ac_rad", 0, true);// [rad] x-axis angle of prop system
	anglesProp_ac_rad_node[1]= fgGetNode(ac, (char *)"/anglesProp_ac_rad", 1, true);// [rad] alpha angle of prop system
	anglesProp_ac_rad_node[2]= fgGetNode(ac, (char *)"/anglesProp_ac_rad", 2, true);// [rad] beta angle of prop system
	rCG_ac_m_node[0] = fgGetNode(ac, (char *)"/rCG_ac_m", 0, true);			// [m] x location of CG
	rCG_ac_m_node[1] = fgGetNode(ac, (char *)"/rCG_ac_m", 1, true);			// [m] y location of CG
	rCG_ac_m_node[2] = fgGetNode(ac, (char *)"/rCG_ac_m", 2, true);			// [m] z location of CG
	prop_radius_ac_m_node = fgGetNode(ac, (char *)"/prop_radius_ac_m", 0, true);// [m] propeller radius
	for(int k=0; k < 5; k++)
	{
		prop_CT_ac_node[k] = fgGetNode(ac, (char *)"/prop_CT_ac", k, true);	// [non-dim], Thrust coefficients
	}
	Nz_lim_ac_g_node = fgGetNode(ac, (char *)"/Nz_lim_ac_g", 0, true);		// [G] aircraft load limit
	alpha_lim_ac_rad_node = fgGetNode(ac, (char *)"/alpha_lim_ac_rad", 0, true);// [rad] angle of attack limit
	beta_lim_ac_rad_node = fgGetNode(ac, (char *)"/beta_lim_ac_rad", 0, true);// [rad] sideslip limit
	p_lim_ac_node = fgGetNode(ac, (char *)"/p_lim_ac", 0, true);			// [non-dim] roll rate limit (p*b)/(2*V)
	q_lim_ac_node = fgGetNode(ac, (char *)"/q_lim_ac", 0, true);			// [non-dim] pitch rate limit (p*b)/(2*V)
	r_lim_ac_node = fgGetNode(ac, (char *)"/r_lim_ac", 0, true);			// [non-dim] yaw rate limit (p*b)/(2*V)
	phi_lim_ac_rad_node = fgGetNode(ac, (char *)"/phi_lim_ac_rad", 0, true);// [rad] bank angle limit


	// ======= Initialize local output variables =======
	// Aircraft variables
	Cl_beta_ac = 0.0;													// [non-dim], Roll moment coefficient (beta)
	Cl_p_ac = 0.0;														// [non-dim], Roll moment coefficient (p)
	Cl_r_ac = 0.0;														// [non-dim], Roll moment coefficient (r)
	Cm_alpha_ac = 0.0;													// [non-dim], Pitch moment coefficient (alpha)
	Cm_q_ac = 0.0;														// [non-dim], Pitch moment coefficient (q)
	Cm_zero_ac = 0.0;													// [non-dim], Pitch moment coefficient (zero)
	Cn_beta_ac = 0.0;													// [non-dim], Yaw moment coefficient (beta)
	Cn_p_ac = 0.0;														// [non-dim], Yaw moment coefficient (p)
	Cn_r_ac = 0.0;														// [non-dim], Yaw moment coefficient (r)
	for(int i=0; i < 20; i++)
	{
		Cl_surf_ac[i] = 0.0;											// [non-dim], Roll moment coefficients (surfaces)
		Cm_surf_ac[i] = 0.0;											// [non-dim], Pitch moment coefficients (surfaces)
		Cn_surf_ac[i] = 0.0; 											// [non-dim], Yaw moment coefficients (surfaces)
		PWMOut_ac[i] = 0;												// [non-dim], PWM Output mapping
	}
	CL_alpha = 0.0;														// [non-dim], Lift (alpha) coefficient
	wingArea_ac_m2 = 0.0;												// [m^2], aircraft wing area
	wingSpan_ac_m = 0.0;												// [m], aircraft wingspan
	wingChord_ac_m = 0.0;												// [m], aircraft wing chord
	Ixx_ac_kgm2 = 0.0;													// [kg*m^2] moment of inertia
	Ixz_ac_kgm2 = 0.0;													// [kg*m^2] moment of inertia
	Iyy_ac_kgm2 = 0.0;													// [kg*m^2] moment of inertia
	Izz_ac_kgm2 = 0.0;													// [kg*m^2] moment of inertia
	mass_ac_kg = 0.0;													// [kg] aircraft's mass
	prop_radius_ac_m = 0.0;												// [m] propeller radius
	for(int j=0; j < 3; j++)
	{
		rProp_ac_m[j] = 0.0;											// [m] location of prop system
		anglesProp_ac_rad[j] = 0.0;										// [rad] angle of prop system
		rCG_ac_m[j] = 0.0;												// [m] location of CG in body (x,y,z)
	}
	for(int k=0; k < 5; k++)
	{
		prop_CT_ac[k] = 0.0;											// [non-dim], Thrust coefficients
	}
	Nz_lim_ac_g = 0.0;													// [G] aircraft load limit
	alpha_lim_ac_rad = 0.0;												// [rad] angle of attack limit
	beta_lim_ac_rad = 0.0;												// [rad] sideslip limit
	p_lim_ac = 0.0;														// [non-dim] roll rate limit (p*b)/(2*V)
	q_lim_ac = 0.0;														// [non-dim] pitch rate limit (q*c)/(2*V)
	r_lim_ac = 0.0;														// [non-dim] yaw rate limit (r*b)/(2*V)
	phi_lim_ac_rad = 0.0;												// [rad] bank angle limit

	// ======= HDF5 identifiers =======
	hid_t		file_id; 	// File identifier
	hid_t		group; 		// Root group id
	herr_t 		status; 	// Returned err status
	ssize_t 	len;		// name length
	hid_t 		grpid;		// subgroup id

	// ======= Name identifiers =======
	char group_name[512]; 	// Group name
	char *mem_name;			// Member name

	// ======= Open h5 file =======
	file_id = H5Fopen(FILE, H5F_ACC_RDONLY, H5P_DEFAULT);
	if (file_id < 0)
	{
		printf("Unable to open file\n");
		while(1)
			;
	}

	// ======= Read h5 values =======
	// Open root group
	group = H5Gopen(file_id, "/", H5P_DEFAULT);

	// Get Checksum
	int Checksum[2] = {0, 0};
	do_dset((char *)"Checksum", group, Checksum);
	printf("Checksum vals are: %d %d\n", Checksum[0], Checksum[1]);

	// Get mass
	do_dset((char *)"Mass", group, &mass_ac_kg);
	printf("mass is: %f\n", mass_ac_kg);

	// Get PWM Output mapping
	do_dset((char *)"PWMOutput", group, &PWMOut_ac);
	printf("PWM Output is: %d\n", PWMOut_ac[15]);

	// Get Cl_alpha
	snprintf(group_name, 512, "/Aero/CL");
	grpid = H5Gopen(file_id, group_name, H5P_DEFAULT);
	do_dset((char *)"alpha", grpid, &CL_alpha);
	printf("CL_alpha is: %f\n", CL_alpha);

	// Get Cl_ac
	snprintf(group_name, 512, "/Aero/Cl");
	grpid = H5Gopen(file_id, group_name, H5P_DEFAULT);
	do_dset((char *)"beta", grpid, &Cl_beta_ac);
	do_dset((char *)"p", grpid, &Cl_p_ac);
	do_dset((char *)"r", grpid, &Cl_r_ac);
	do_dset((char *)"surf", grpid, &Cl_surf_ac);
	printf("Cl_beta is: %f\n", Cl_beta_ac);
	printf("Cl_p is: %f\n", Cl_p_ac);
	printf("Cl_r is: %f\n", Cl_r_ac);
	printf("Cl_surf 1 is: %f\n", Cl_surf_ac[0]);

	// Get Cm_ac
	snprintf(group_name, 512, "/Aero/Cm");
	grpid = H5Gopen(file_id, group_name, H5P_DEFAULT);
	do_dset((char *)"alpha", grpid, &Cm_alpha_ac);
	do_dset((char *)"q", grpid, &Cm_q_ac);
	do_dset((char *)"zero", grpid, &Cm_zero_ac);
	do_dset((char *)"surf", grpid, &Cm_surf_ac);
	printf("Cm_alpha is: %f\n", Cm_alpha_ac);
	printf("Cm_q is: %f\n", Cm_q_ac);
	printf("Cm_zero is: %f\n", Cm_zero_ac);

	// Get Cn_ac
	snprintf(group_name, 512, "/Aero/Cn");
	grpid = H5Gopen(file_id, group_name, H5P_DEFAULT);
	do_dset((char *)"beta", grpid, &Cn_beta_ac);
	do_dset((char *)"p", grpid, &Cn_p_ac);
	do_dset((char *)"r", grpid, &Cn_r_ac);
	do_dset((char *)"surf", grpid, &Cn_surf_ac);
	printf("Cn_beta is: %f\n", Cn_beta_ac);
	printf("Cn_p is: %f\n", Cn_p_ac);
	printf("Cn_r is: %f\n", Cn_r_ac);

	// Get geometry
	snprintf(group_name, 512, "/Geometry");
	grpid = H5Gopen(file_id, group_name, H5P_DEFAULT);
	do_dset((char *)"S", grpid, &wingArea_ac_m2);
	do_dset((char *)"b", grpid, &wingSpan_ac_m);
	do_dset((char *)"c", grpid, &wingChord_ac_m);
	do_dset((char *)"rProp", grpid, &rProp_ac_m);
	do_dset((char *)"anglesProp", grpid, &anglesProp_ac_rad);
	do_dset((char *)"rCG", grpid, &rCG_ac_m);
	printf("S is: %f\n", wingArea_ac_m2);
	printf("b is: %f\n", wingSpan_ac_m);
	printf("c is: %f\n", wingChord_ac_m);
	printf("xCG is: %f\n",rCG_ac_m[0]);
	printf("xProp is: %f\n", rProp_ac_m[0]);
	printf("beta Prop is: %f\n", anglesProp_ac_rad[2]);

	// Get Inertia
	snprintf(group_name, 512, "/Inertia");
	grpid = H5Gopen(file_id, group_name, H5P_DEFAULT);
	do_dset((char *)"Ixx", grpid, &Ixx_ac_kgm2);
	do_dset((char *)"Ixz", grpid, &Ixz_ac_kgm2);
	do_dset((char *)"Iyy", grpid, &Iyy_ac_kgm2);
	do_dset((char *)"Izz", grpid, &Izz_ac_kgm2);
	printf("Ixx is: %f\n", Ixx_ac_kgm2);
	printf("Ixz is: %f\n", Ixz_ac_kgm2);
	printf("Iyy is: %f\n", Iyy_ac_kgm2);
	printf("Izz is: %f\n", Izz_ac_kgm2);

	// Get prop
	snprintf(group_name, 512, "/Prop");
	grpid = H5Gopen(file_id, group_name, H5P_DEFAULT);
	do_dset((char *)"radius", grpid, &prop_radius_ac_m);
	do_dset((char *)"CT", grpid, &prop_CT_ac);
	printf("Prop radius is: %f\n", prop_radius_ac_m);
	printf("CT (3) is: %f\n", prop_CT_ac[2]);

	// Get Flight Envelope
	snprintf(group_name, 512, "/Envelope");
	grpid = H5Gopen(file_id, group_name, H5P_DEFAULT);
	do_dset((char *)"Nz", grpid, &Nz_lim_ac_g);
	do_dset((char *)"alpha", grpid, &alpha_lim_ac_rad);
	do_dset((char *)"beta", grpid, &beta_lim_ac_rad);
	do_dset((char *)"p", grpid, &p_lim_ac);
	do_dset((char *)"q", grpid, &q_lim_ac);
	do_dset((char *)"r", grpid, &r_lim_ac);
	do_dset((char *)"phi", grpid, &phi_lim_ac_rad);
	printf("Load limit is: %f\n", Nz_lim_ac_g);
	printf("Alpha limit is: %f\n", alpha_lim_ac_rad);
	printf("Beta limit is: %f\n", beta_lim_ac_rad);
	printf("p limit is: %f\n", p_lim_ac);
	printf("q limit is: %f\n", q_lim_ac);
	printf("r limit is: %f\n", r_lim_ac);
	printf("Bank limit is: %f\n", phi_lim_ac_rad);

	// ======= Set output variables to SGProps =======
	printf("Setting SGProps output values\n");
	Cl_beta_ac_node->setDoubleValue(Cl_beta_ac);						// [non-dim], Cl_beta
	Cl_p_ac_node->setDoubleValue(Cl_p_ac);								// [non-dim], Cl_p
	Cl_r_ac_node->setDoubleValue(Cl_r_ac);								// [non-dim], Cl_r
	Cm_alpha_ac_node->setDoubleValue(Cm_alpha_ac);						// [non-dim], Cm_alpha
	Cm_q_ac_node->setDoubleValue(Cm_q_ac);								// [non-dim], Cm_q
	Cm_zero_ac_node->setDoubleValue(Cm_zero_ac);						// [non-dim], Cm_zero
	Cn_beta_ac_node->setDoubleValue(Cn_beta_ac);						// [non-dim], Cn_beta
	Cn_p_ac_node->setDoubleValue(Cn_p_ac);								// [non-dim], Cn_p
	Cn_r_ac_node->setDoubleValue(Cn_r_ac);								// [non-dim], Cn_r
	for(int i=0; i < 20; i++)
	{
		Cl_surf_ac_node[i]->setDoubleValue(Cl_surf_ac[i]);				// [non-dim], Cl_surfaces
		Cm_surf_ac_node[i]->setDoubleValue(Cm_surf_ac[i]);				// [non-dim], Cm_surfaces
		Cn_surf_ac_node[i]->setDoubleValue(Cn_surf_ac[i]);				// [non-dim], Cn_surfaces
		PWMOut_ac_node[i]->setIntValue(PWMOut_ac[i]);					// [non-dim], PWM Output mapping
	}
	CL_alpha_node->setDoubleValue(CL_alpha);							// [non-dim], Lift (alpha) coefficient
	wingArea_ac_m2_node->setDoubleValue(wingArea_ac_m2);				// [m^2], aircraft wing area
	wingSpan_ac_m_node->setDoubleValue(wingSpan_ac_m);					// [m], aircraft wingspan
	wingChord_ac_m_node->setDoubleValue(wingChord_ac_m);				// [m], aircraft wing chord
	Ixx_ac_kgm2_node->setDoubleValue(Ixx_ac_kgm2);						// [kg*m^2] moment of inertia
	Ixz_ac_kgm2_node->setDoubleValue(Ixz_ac_kgm2);						// [kg*m^2] moment of inertia
	Iyy_ac_kgm2_node->setDoubleValue(Iyy_ac_kgm2);						// [kg*m^2] moment of inertia
	Izz_ac_kgm2_node->setDoubleValue(Izz_ac_kgm2);						// [kg*m^2] moment of inertia
	mass_ac_kg_node->setDoubleValue(mass_ac_kg);						// [kg] aircraft's mass
	prop_radius_ac_m_node->setDoubleValue(prop_radius_ac_m);			// [m] propeller radius
	for(int j=0; j < 3; j++)
	{
		rProp_ac_m_node[j]->setDoubleValue(rProp_ac_m[j]);				// [m] location of prop system
		anglesProp_ac_rad_node[j]->setDoubleValue(anglesProp_ac_rad[j]);// [rad] angle of prop system
		rCG_ac_m_node[j]->setDoubleValue(rCG_ac_m[j]);					// [m] location of CG in body (x,y,z)
	}
	for(int k=0; k < 5; k++)
	{
		prop_CT_ac_node[k]->setDoubleValue(prop_CT_ac[k]);				// [non-dim], Thrust coefficients
	}
	Nz_lim_ac_g_node->setDoubleValue(Nz_lim_ac_g);						// [G] aircraft load limit
	alpha_lim_ac_rad_node->setDoubleValue(alpha_lim_ac_rad);			// [rad] angle of attack limit
	beta_lim_ac_rad_node->setDoubleValue(beta_lim_ac_rad);				// [rad] sideslip limit
	p_lim_ac_node->setDoubleValue(p_lim_ac);							// [non-dim] roll rate limit (p*b)/(2*V)
	q_lim_ac_node->setDoubleValue(q_lim_ac);							// [non-dim] pitch rate limit (q*c)/(2*V)
	r_lim_ac_node->setDoubleValue(r_lim_ac);							// [non-dim] yaw rate limit (r*b)/(2*V)
	phi_lim_ac_rad_node->setDoubleValue(phi_lim_ac_rad);				// [rad] bank angle limit


	printf("SGProps updated\n");

	// ======= Checksum =======
	unsigned char CK[2];
	for (int i=0; i < 20; i++)
	{
		capePacket.Cl_surf[i] = Cl_surf_ac[i];
		capePacket.Cm_surf[i] = Cm_surf_ac[i];
		capePacket.Cn_surf[i] = Cl_surf_ac[i];
		capePacket.PWMOut[i] = PWMOut_ac[i];
	}
	capePacket.Cl_beta = Cl_beta_ac;
	capePacket.Cl_p = Cl_p_ac;
	capePacket.Cl_r = Cl_r_ac;
	capePacket.Cm_alpha = Cm_alpha_ac;
	capePacket.Cm_q = Cm_q_ac;
	capePacket.Cm_zero = Cm_zero_ac;
	capePacket.Cn_beta = Cn_beta_ac;
	capePacket.Cn_p = Cn_p_ac;
	capePacket.Cn_r = Cn_r_ac;
	capePacket.CL_alpha = CL_alpha;
	capePacket.wingArea = wingArea_ac_m2;
	capePacket.wingChord = wingChord_ac_m;
	capePacket.wingSpan = wingSpan_ac_m;
	capePacket.Ixx = Ixx_ac_kgm2;
	capePacket.Ixz = Ixz_ac_kgm2;
	capePacket.Iyy = Iyy_ac_kgm2;
	capePacket.Izz = Izz_ac_kgm2;
	capePacket.mass = mass_ac_kg;
	for (int j=0; j < 3; j++)
	{
		capePacket.rCG[j] = rCG_ac_m[j];
		capePacket.rProp[j] = rProp_ac_m[j];
		capePacket.anglesProp[j] = anglesProp_ac_rad[j];
	}
	capePacket.prop_radius = prop_radius_ac_m;
	for(int k=0; k < 5; k++)
	{
		capePacket.prop_CT[k] = prop_CT_ac[k];				// [non-dim], Thrust coefficients
	}
	capePacket.Nz_lim = Nz_lim_ac_g;
	capePacket.alpha_lim = alpha_lim_ac_rad;
	capePacket.beta_lim = beta_lim_ac_rad;
	capePacket.p_lim = p_lim_ac;
	capePacket.q_lim = q_lim_ac;
	capePacket.r_lim = r_lim_ac;
	capePacket.phi_lim = phi_lim_ac_rad;
	calcCapeChecksum(CK);
	printf("The checksum is %u %u\n", CK[0],CK[1]);
	// Compare checksum
	if ((CK[0] != Checksum[0]) || (CK[1] != Checksum[1]))
	{
		printf("==================================================\n");
		printf("ERROR: Checksum mismatch! ... exiting\n");
		printf("==================================================\n");
		exit(-1);
	}


	// ======= Close h5 file =======
	status = H5Fclose(file_id);
	if (status < 0)
	{
		printf("Error closing config file\n");
		exit(-1);
	}

}

void do_dset(char *mem_name, hid_t grpid, void *mem)
{
	hid_t 	dsid;			// dataset id
	hid_t	sid;			// dataspace id
	hid_t	tid;			// datatype id
	herr_t	status;			// returned error status
	H5T_class_t t_class;	// data class
	//double buf;				// data buffer

	// ======= Open dataset =======
	dsid = H5Dopen(grpid, mem_name, H5P_DEFAULT);
	if (dsid < 0)
	{
		printf("ERROR: Member not found -> %s\n", mem_name);
	}

	// ======= Get dataspace id =======
	sid = H5Dget_space(dsid);
	if (sid < 0)
	{
		printf("ERROR: Get data space failed -> %s\n", mem_name);
	}
	// ======= Get datatype =======
	tid = H5Dget_type(dsid);
	if (tid < 0)
	{
		printf("ERROR: Get data type failed -> %s\n", mem_name);
	}

	// ======= Read the dataset value =======
	status = H5Dread(dsid, tid, sid, H5S_ALL, H5P_DEFAULT, mem);
	if (status < 0)
	{
		printf("Read %s error!\n", mem_name);
	}

	// ======= Close all identifiers =======
	H5Tclose(tid);
	H5Sclose(sid);
	H5Dclose(dsid);
}

void calcCapeChecksum(unsigned char* CK)
{
	CK[0] = 0;
	CK[1] = 1;
	for(int i=0; i < (int)sizeof(capePacket); i++)
	{
		CK[0] += ((unsigned char*)(&capePacket))[i];
		CK[1] += CK[0];
	}
}
