/*
 * aircraft_config.cxx
 *
 *  Created on: Feb 25, 2016
 *      Author: patipan
 */

#include "hdf5.h"
#include <stdio.h>
#include "aircraft_config.hxx"

#define FILE "UltraStick25e_config.mat"

double do_dset(char *, hid_t);

double Cl_array[6];
double Cm_array[5];
double Cn_array[6];
double CL_alpha;
double S; // Wing area [m^2]
double b; // wing span [m]
double c; // wing chord [m]

int main()
{
	hid_t		file_id; // File identifier
	hid_t		group;
	herr_t 		status;

	file_id = H5Fopen(FILE, H5F_ACC_RDONLY, H5P_DEFAULT);
	group = H5Gopen(file_id, "/", H5P_DEFAULT);
	get_config(group);

	status = H5Fclose(file_id);
	if (status < 0)
	{
		printf("Error closing config file\n");
		return -1;
	}

	return 0;
}

void get_config(hid_t group)
{
	char group_name[512];
	char parent_name[30];
	char *mem_name;
	double val;
	ssize_t 	len;
	hsize_t 	nobj;
	hid_t 		grpid;
	hid_t 		dsid;
	hid_t		sid;
	herr_t 		status;

	// Get root group name
	len = H5Gget_objname_by_idx(group, (hsize_t)0, parent_name, 30);
	grpid = H5Gopen(group, parent_name, H5P_DEFAULT);
	len = H5Iget_name(grpid, parent_name, 30);
	printf("Parent group name: %s\n", parent_name);

	// Get Cl_alpha
	snprintf(group_name, 512, "%s%s", parent_name, "/Aero/CL");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	val = do_dset((char *)"alpha", grpid);
	CL_alpha = val;
	printf("CL_alpha is: %f\n", CL_alpha);

	// Get Cl_array
	snprintf(group_name, 512, "%s%s", parent_name, "/Aero/Cl");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	val = do_dset((char *)"beta", grpid);
	Cl_array[0] = val;
	val = do_dset((char *)"p", grpid);
	Cl_array[1] = val;
	val = do_dset((char *)"r", grpid);
	Cl_array[2] = val;
	val = do_dset((char *)"dail", grpid);
	Cl_array[3] = val;
	val = do_dset((char *)"drud", grpid);
	Cl_array[4] = val;
	val = do_dset((char *)"dflap", grpid);
	Cl_array[5] = val;
	printf("Cl_beta is: %f\n", Cl_array[0]);
	printf("Cl_p is: %f\n", Cl_array[1]);
	printf("Cl_r is: %f\n", Cl_array[2]);
	printf("Cl_dail is: %f\n", Cl_array[3]);
	printf("Cl_drud is: %f\n", Cl_array[4]);
	printf("Cl_dflap is: %f\n", Cl_array[5]);

	// Get Cm_array
	snprintf(group_name, 512, "%s%s", parent_name, "/Aero/Cm");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	val = do_dset((char *)"alpha", grpid);
	Cm_array[0] = val;
	val = do_dset((char *)"q", grpid);
	Cm_array[1] = val;
	val = do_dset((char *)"delev", grpid);
	Cm_array[2] = val;
	val = do_dset((char *)"dflap", grpid);
	Cm_array[3] = val;
	val = do_dset((char *)"dail", grpid);
	Cm_array[4] = val;
	printf("Cm_alpha is: %f\n", Cm_array[0]);
	printf("Cm_q is: %f\n", Cm_array[1]);
	printf("Cm_delev is: %f\n", Cm_array[2]);
	printf("Cm_dflap is: %f\n", Cm_array[3]);
	printf("Cm_dail is: %f\n", Cm_array[4]);

	// Get Cn_array
	snprintf(group_name, 512, "%s%s", parent_name, "/Aero/Cn");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	val = do_dset((char *)"beta", grpid);
	Cn_array[0] = val;
	val = do_dset((char *)"p", grpid);
	Cn_array[1] = val;
	val = do_dset((char *)"r", grpid);
	Cn_array[2] = val;
	val = do_dset((char *)"dail", grpid);
	Cn_array[3] = val;
	val = do_dset((char *)"drud", grpid);
	Cn_array[4] = val;
	val = do_dset((char *)"dflap", grpid);
	Cn_array[5] = val;
	printf("Cn_beta is: %f\n", Cn_array[0]);
	printf("Cn_p is: %f\n", Cn_array[1]);
	printf("Cn_r is: %f\n", Cn_array[2]);
	printf("Cn_dail is: %f\n", Cn_array[3]);
	printf("Cn_drud is: %f\n", Cn_array[4]);
	printf("Cn_dflap is: %f\n", Cn_array[5]);
}

double do_dset(char *mem_name, hid_t grpid)
{
	hid_t 	dsid;
	hid_t	sid;
	herr_t	status;
	double buf;

	dsid = H5Dopen(grpid, mem_name, H5P_DEFAULT);
	sid = H5Dget_space(dsid);
	status = H5Dread(dsid, H5T_NATIVE_DOUBLE, sid, H5S_ALL, H5P_DEFAULT, &buf);
	return buf;
}
