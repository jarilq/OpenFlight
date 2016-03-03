/*
 * aircraft_config.cxx
 *
 *  Created on: Feb 25, 2016
 *      Author: patipan
 */

#include "ac_config.hxx"

#include "hdf5.h"
#include <stdio.h>
#include "../props.hxx"
#include <cryptopp/sha.h>
#include <cryptopp/files.h>
#include <cryptopp/hex.h>

#define FILE "aircraft/UltraStick25e_config.mat"

// ======= Local Function =======
static double do_dset(char *, hid_t);

// ======= Output Properties =======
// Aircraft Characteristics UltraStick25e variables
static SGPropertyNode *Cl_ac_node[6] = {NULL, NULL, NULL, NULL, NULL, NULL};// [non-dim], Roll moment coefficients
static SGPropertyNode *Cm_ac_node[5] = {NULL, NULL, NULL, NULL, NULL};		// [non-dim], Pitch moment coefficients
static SGPropertyNode *Cn_ac_node[6] = {NULL, NULL, NULL, NULL, NULL, NULL};// [non-dim], Yaw moment coefficients
static SGPropertyNode *CL_alpha_node = NULL;								// [non-dim], Lift (alpha) coefficient
static SGPropertyNode *wingArea_ac_m2_node = NULL;							// [m^2], aircraft wing area
static SGPropertyNode *wingSpan_ac_m_node = NULL;							// [m], aircraft wingspan
static SGPropertyNode *wingChord_ac_m_node = NULL;							// [m], aircraft wing chord
static SGPropertyNode *Ixx_ac_kgm2_node = NULL;								// [kg*m^2] moment of inertia
static SGPropertyNode *Ixz_ac_kgm2_node = NULL;								// [kg*m^2] moment of inertia
static SGPropertyNode *Iyy_ac_kgm2_node = NULL;								// [kg*m^2] moment of inertia
static SGPropertyNode *Izz_ac_kgm2_node = NULL;								// [kg*m^2] moment of inertia

// ======= Output Local Variables =======
static double Cl_ac[6];														// [non-dim], Roll moment coefficients
static double Cm_ac[5];														// [non-dim], Pitch moment coefficients
static double Cn_ac[6];														// [non-dim], Yaw moment coefficients
static double CL_alpha;														// [non-dim], Lift (alpha) coefficient
static double wingArea_ac_m2;												// [m^2], aircraft wing area
static double wingSpan_ac_m;												// [m], aircraft wingspan
static double wingChord_ac_m;												// [m], aircraft wing chord
static double Ixx_ac_kgm2;													// [kg*m^2] moment of inertia
static double Ixz_ac_kgm2;													// [kg*m^2] moment of inertia
static double Iyy_ac_kgm2;													// [kg*m^2] moment of inertia
static double Izz_ac_kgm2;													// [kg*m^2] moment of inertia


void get_config()
{
	// ======= Variables for concatenating path =======
	char ac[] = "/ac";
	char imu[] = "/imu";
	char nav[] = "/nav";
	char ad[] = "/ad";
	char ctrl[] = "/ctrl";

	// ======= Property node initialization =======
	// Aircraft variables
	Cl_ac_node[0] = fgGetNode(ac, (char *)"/Cl_ac", 0, true);				// [non-dim], Cl_beta
	Cl_ac_node[1] = fgGetNode(ac, (char *)"/Cl_ac", 1, true);				// [non-dim], Cl_p
	Cl_ac_node[2] = fgGetNode(ac, (char *)"/Cl_ac", 2, true);				// [non-dim], Cl_r
	Cl_ac_node[3] = fgGetNode(ac, (char *)"/Cl_ac", 3, true);				// [non-dim], Cl_dail
	Cl_ac_node[4] = fgGetNode(ac, (char *)"/Cl_ac", 4, true);				// [non-dim], Cl_drud
	Cl_ac_node[5] = fgGetNode(ac, (char *)"/Cl_ac", 5, true);				// [non-dim], Cl_dflap
	Cm_ac_node[0] = fgGetNode(ac, (char *)"/Cm_ac", 0, true);				// [non-dim], Cm_alpha
	Cm_ac_node[1] = fgGetNode(ac, (char *)"/Cm_ac", 1, true);				// [non-dim], Cm_q
	Cm_ac_node[2] = fgGetNode(ac, (char *)"/Cm_ac", 2, true);				// [non-dim], Cm_delev
	Cm_ac_node[3] = fgGetNode(ac, (char *)"/Cm_ac", 3, true);				// [non-dim], Cm_dflap
	Cm_ac_node[4] = fgGetNode(ac, (char *)"/Cm_ac", 4, true);				// [non-dim], Cm_dail
	Cn_ac_node[0] = fgGetNode(ac, (char *)"/Cn_ac", 0, true);				// [non-dim], Cn_beta
	Cn_ac_node[1] = fgGetNode(ac, (char *)"/Cn_ac", 1, true);				// [non-dim], Cn_p
	Cn_ac_node[2] = fgGetNode(ac, (char *)"/Cn_ac", 2, true);				// [non-dim], Cn_r
	Cn_ac_node[3] = fgGetNode(ac, (char *)"/Cn_ac", 3, true);				// [non-dim], Cn_dail
	Cn_ac_node[4] = fgGetNode(ac, (char *)"/Cn_ac", 4, true);				// [non-dim], Cn_drud
	Cn_ac_node[5] = fgGetNode(ac, (char *)"/Cn_ac", 5, true);				// [non-dim], Cn_dflap
	CL_alpha_node = fgGetNode(ac, (char *)"/CL_alpha", 0, true);			// [non-dim], Lift (alpha) coefficient
	wingArea_ac_m2_node = fgGetNode(ac, (char *)"/wingArea_ac_m2", 0, true);// [m^2], aircraft wing area
	wingSpan_ac_m_node = fgGetNode(ac, (char *)"/wingSpan_ac_m", 0, true);	// [m], aircraft wingspan
	wingChord_ac_m_node = fgGetNode(ac, (char *)"/wingChord_ac_m", 0, true);// [m], aircraft wing chord
	Ixx_ac_kgm2_node = fgGetNode(ac, (char *)"/Ixx_ac_kgm2", 0, true);		// [kg*m^2] moment of inertia
	Ixz_ac_kgm2_node = fgGetNode(ac, (char *)"/Ixz_ac_kgm2", 0, true);		// [kg*m^2] moment of inertia
	Iyy_ac_kgm2_node = fgGetNode(ac, (char *)"/Iyy_ac_kgm2", 0, true);		// [kg*m^2] moment of inertia
	Izz_ac_kgm2_node = fgGetNode(ac, (char *)"/Izz_ac_kgm2", 0, true);		// [kg*m^2] moment of inertia


	// ======= Initialize local output variables =======
	// Aircraft variables
	Cl_ac[0] = 0.0;															// [non-dim], Cl_beta
	Cl_ac[1] = 0.0;															// [non-dim], Cl_p
	Cl_ac[2] = 0.0;															// [non-dim], Cl_r
	Cl_ac[3] = 0.0;															// [non-dim], Cl_dail
	Cl_ac[4] = 0.0;															// [non-dim], Cl_drud
	Cl_ac[5] = 0.0;															// [non-dim], Cl_dflap
	Cm_ac[0] = 0.0;															// [non-dim], Cm_alpha
	Cm_ac[1] = 0.0;															// [non-dim], Cm_q
	Cm_ac[2] = 0.0;															// [non-dim], Cm_delev
	Cm_ac[3] = 0.0;															// [non-dim], Cm_dflap
	Cm_ac[4] = 0.0;															// [non-dim], Cm_dail
	Cn_ac[0] = 0.0;															// [non-dim], Cn_beta
	Cn_ac[1] = 0.0;															// [non-dim], Cn_p
	Cn_ac[2] = 0.0;															// [non-dim], Cn_q
	Cn_ac[3] = 0.0;															// [non-dim], Cn_dail
	Cn_ac[4] = 0.0;															// [non-dim], Cn_drud
	Cn_ac[5] = 0.0;															// [non-dim], Cn_dflap
	CL_alpha = 0.0;															// [non-dim], Lift (alpha) coefficient
	wingArea_ac_m2 = 0.0;													// [m^2], aircraft wing area
	wingSpan_ac_m = 0.0;													// [m], aircraft wingspan
	wingChord_ac_m = 0.0;													// [m], aircraft wing chord
	Ixx_ac_kgm2 = 0.0;														// [kg*m^2] moment of inertia
	Ixz_ac_kgm2 = 0.0;														// [kg*m^2] moment of inertia
	Iyy_ac_kgm2 = 0.0;														// [kg*m^2] moment of inertia
	Izz_ac_kgm2 = 0.0;														// [kg*m^2] moment of inertia

	// ======= HDF5 identifiers =======
	hid_t		file_id; 	// File identifier
	hid_t		group; 		// Root group id
	herr_t 		status; 	// Returned err status
	ssize_t 	len;		// name length
	hid_t 		grpid;		// subgroup id

	// ======= Name identifiers =======
	char group_name[512]; 	// Group name
	char parent_name[30];	// Name of parent group
	char *mem_name;			// Member name

	// ======= Check .matfile =======
	CryptoPP::SHA1 sha1;	// Create SHA1 instance
	string SHA_result;		// checksum result
	// Get checksum
//	CryptoPP::FileSource(FILE, true, new CryptoPP::HashFilter(sha1,
//			new CryptoPP::HexEncoder(
//			new CryptoPP::StringSink(SHA_result), true)));
	CryptoPP::FileSource("aircraft/UltraStick25e_config.mat", true,
			new CryptoPP::HashFilter(sha1,
			new CryptoPP::HexEncoder(
			new CryptoPP::StringSink(SHA_result), true)));
	std::cout << "Checksum of file is: " << SHA_result << std::endl;

	// ======= Open .mat file =======
	file_id = H5Fopen(FILE, H5F_ACC_RDONLY, H5P_DEFAULT);

	// ======= Read .mat values =======
	// Open root group
	group = H5Gopen(file_id, "/", H5P_DEFAULT);
	// Get root group name
	len = H5Gget_objname_by_idx(group, (hsize_t)0, parent_name, 30);
	grpid = H5Gopen(group, parent_name, H5P_DEFAULT);
	len = H5Iget_name(grpid, parent_name, 30);
	printf("Parent group name: %s\n", parent_name);

	// Get Cl_alpha
	snprintf(group_name, 512, "%s%s", parent_name, "/Aero/CL");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	CL_alpha = do_dset((char *)"alpha", grpid);
	printf("CL_alpha is: %f\n", CL_alpha);

	// Get Cl_ac
	snprintf(group_name, 512, "%s%s", parent_name, "/Aero/Cl");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	Cl_ac[0] = do_dset((char *)"beta", grpid);
	Cl_ac[1] = do_dset((char *)"p", grpid);
	Cl_ac[2] = do_dset((char *)"r", grpid);
	Cl_ac[3] = do_dset((char *)"dail", grpid);
	Cl_ac[4] = do_dset((char *)"drud", grpid);
	Cl_ac[5] = do_dset((char *)"dflap", grpid);
	printf("Cl_beta is: %f\n", Cl_ac[0]);
	printf("Cl_p is: %f\n", Cl_ac[1]);
	printf("Cl_r is: %f\n", Cl_ac[2]);
	printf("Cl_dail is: %f\n", Cl_ac[3]);
	printf("Cl_drud is: %f\n", Cl_ac[4]);
	printf("Cl_dflap is: %f\n", Cl_ac[5]);

	// Get Cm_ac
	snprintf(group_name, 512, "%s%s", parent_name, "/Aero/Cm");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	Cm_ac[0] = do_dset((char *)"alpha", grpid);
	Cm_ac[1] = do_dset((char *)"q", grpid);
	Cm_ac[2] = do_dset((char *)"delev", grpid);
	Cm_ac[3] = do_dset((char *)"dflap", grpid);
	Cm_ac[4] = do_dset((char *)"dail", grpid);
	printf("Cm_alpha is: %f\n", Cm_ac[0]);
	printf("Cm_q is: %f\n", Cm_ac[1]);
	printf("Cm_delev is: %f\n", Cm_ac[2]);
	printf("Cm_dflap is: %f\n", Cm_ac[3]);
	printf("Cm_dail is: %f\n", Cm_ac[4]);

	// Get Cn_ac
	snprintf(group_name, 512, "%s%s", parent_name, "/Aero/Cn");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	Cn_ac[0] = do_dset((char *)"beta", grpid);
	Cn_ac[1] = do_dset((char *)"p", grpid);
	Cn_ac[2] = do_dset((char *)"r", grpid);
	Cn_ac[3] = do_dset((char *)"dail", grpid);
	Cn_ac[4] = do_dset((char *)"drud", grpid);
	Cn_ac[5] = do_dset((char *)"dflap", grpid);
	printf("Cn_beta is: %f\n", Cn_ac[0]);
	printf("Cn_p is: %f\n", Cn_ac[1]);
	printf("Cn_r is: %f\n", Cn_ac[2]);
	printf("Cn_dail is: %f\n", Cn_ac[3]);
	printf("Cn_drud is: %f\n", Cn_ac[4]);
	printf("Cn_dflap is: %f\n", Cn_ac[5]);

	// Get geometry
	snprintf(group_name, 512, "%s%s", parent_name, "/Geometry");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	wingArea_ac_m2 = do_dset((char *)"S", grpid);
	wingSpan_ac_m = do_dset((char *)"b", grpid);
	wingChord_ac_m = do_dset((char *)"c", grpid);
	printf("S is: %f\n", wingArea_ac_m2);
	printf("b is: %f\n", wingSpan_ac_m);
	printf("c is: %f\n", wingChord_ac_m);

	// Get Inertia
	snprintf(group_name, 512, "%s%s", parent_name, "/Inertia");
	grpid = H5Gopen(grpid, group_name, H5P_DEFAULT);
	Ixx_ac_kgm2 = do_dset((char *)"Ixx", grpid);
	Ixz_ac_kgm2 = do_dset((char *)"Ixz", grpid);
	Iyy_ac_kgm2 = do_dset((char *)"Iyy", grpid);
	Izz_ac_kgm2 = do_dset((char *)"Izz", grpid);
	printf("Ixx is: %f\n", Ixx_ac_kgm2);
	printf("Ixz is: %f\n", Ixz_ac_kgm2);
	printf("Iyy is: %f\n", Iyy_ac_kgm2);
	printf("Izz is: %f\n", Izz_ac_kgm2);

	// ======= Set output variables to SGProps =======
	printf("Setting SGProps output values\n");
	Cl_ac_node[0]->setDoubleValue(Cl_ac[0]);						// [non-dim], Cl_beta
	Cl_ac_node[1]->setDoubleValue(Cl_ac[1]);						// [non-dim], Cl_p
	Cl_ac_node[2]->setDoubleValue(Cl_ac[2]);						// [non-dim], Cl_r
	Cl_ac_node[3]->setDoubleValue(Cl_ac[3]);						// [non-dim], Cl_dail
	Cl_ac_node[4]->setDoubleValue(Cl_ac[4]);						// [non-dim], Cl_drud
	Cl_ac_node[5]->setDoubleValue(Cl_ac[5]);						// [non-dim], Cl_dflap
	Cm_ac_node[0]->setDoubleValue(Cm_ac[0]);						// [non-dim], Cm_alpha
	Cm_ac_node[1]->setDoubleValue(Cm_ac[1]);						// [non-dim], Cm_q
	Cm_ac_node[2]->setDoubleValue(Cm_ac[2]);						// [non-dim], Cm_delev
	Cm_ac_node[3]->setDoubleValue(Cm_ac[3]);						// [non-dim], Cm_dflap
	Cm_ac_node[4]->setDoubleValue(Cm_ac[4]);						// [non-dim], Cm_dail
	Cn_ac_node[0]->setDoubleValue(Cn_ac[0]);						// [non-dim], Cn_beta
	Cn_ac_node[1]->setDoubleValue(Cn_ac[1]);						// [non-dim], Cn_p
	Cn_ac_node[2]->setDoubleValue(Cn_ac[2]);						// [non-dim], Cn_r
	Cn_ac_node[3]->setDoubleValue(Cn_ac[3]);						// [non-dim], Cn_dail
	Cn_ac_node[4]->setDoubleValue(Cn_ac[4]);						// [non-dim], Cn_drud
	Cn_ac_node[5]->setDoubleValue(Cn_ac[5]);						// [non-dim], Cn_dflap
	CL_alpha_node->setDoubleValue(CL_alpha);						// [non-dim], Lift (alpha) coefficient
	wingArea_ac_m2_node->setDoubleValue(wingArea_ac_m2);			// [m^2], aircraft wing area
	wingSpan_ac_m_node->setDoubleValue(wingSpan_ac_m);				// [m], aircraft wingspan
	wingChord_ac_m_node->setDoubleValue(wingChord_ac_m);			// [m], aircraft wing chord
	Ixx_ac_kgm2_node->setDoubleValue(Ixx_ac_kgm2);					// [kg*m^2] moment of inertia
	Ixz_ac_kgm2_node->setDoubleValue(Ixz_ac_kgm2);					// [kg*m^2] moment of inertia
	Iyy_ac_kgm2_node->setDoubleValue(Iyy_ac_kgm2);					// [kg*m^2] moment of inertia
	Izz_ac_kgm2_node->setDoubleValue(Izz_ac_kgm2);					// [kg*m^2] moment of inertia

	printf("SGProps updated\n");

	// ======= Close .mat file =======
	status = H5Fclose(file_id);
	if (status < 0)
	{
		printf("Error closing config file\n");
		exit(-1);
	}
}

double do_dset(char *mem_name, hid_t grpid)
{
	hid_t 	dsid;			// dataset id
	hid_t	sid;			// dataspace id
	hid_t	tid;			// datatype id
	herr_t	status;			// returned error status
	H5T_class_t t_class;	// data class
	double buf;				// data buffer

	// ======= Open dataset =======
	dsid = H5Dopen(grpid, mem_name, H5P_DEFAULT);

	// ======= Get dataspace id =======
	sid = H5Dget_space(dsid);

	// ======= Get datatype =======
	tid = H5Dget_type(dsid);

	// ======= Read the dataset value =======
	status = H5Dread(dsid, tid, sid, H5S_ALL, H5P_DEFAULT, &buf);

	// ======= Close all identifiers =======
	H5Tclose(tid);
	H5Sclose(sid);
	H5Dclose(dsid);
	return buf;
}
