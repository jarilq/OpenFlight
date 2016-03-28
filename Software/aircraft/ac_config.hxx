/*
 * aircraft_config.hxx
 *
 *  Created on: Feb 25, 2016
 *      Author: patipan
 */

#ifndef AIRCRAFT_AC_CONFIG_HXX_
#define AIRCRAFT_AC_CONFIG_HXX_

struct cape{
	double Cl_beta;
	double Cl_p;
	double Cl_r;
	double Cl_surf[20];
	double Cm_alpha;
	double Cm_q;
	double Cm_zero;
	double Cm_surf[20];
	double Cn_beta;
	double Cn_p;
	double Cn_r;
	double Cn_surf[20];
	double CL_alpha;
	double wingArea;
	double wingSpan;
	double wingChord;
	double mass;
	double Ixx;
	double Ixz;
	double Iyy;
	double Izz;
	int PWMOut[20];
	double rProp[3];
	double rCG[3];
	double anglesProp[3];
	double prop_radius;
	double prop_CT[5];
	double Nz_lim;
	double alpha_lim;
	double beta_lim;
	double p_lim;
	double q_lim;
	double r_lim;
	double phi_lim;
};

extern void get_config(char *FILE);

#endif /* AIRCRAFT_AC_CONFIG_HXX_ */
