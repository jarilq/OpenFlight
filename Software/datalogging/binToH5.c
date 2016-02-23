/*
 * binToH5.c
 *
 *  Created on: Feb 16, 2016
 *      Author: patipan
 */

#include "hdf5_hl.h"
#include <stdlib.h>
#include <stdio.h>
#include "datalog.h"

#define BIN_FILE	"testData.bin"

int main(void)
{
	// Define h5 variables
	FILE * file;
	hid_t		fid;        // File identifier
	hid_t		ptable;     // Packet table identifier
	hid_t		memtype;	// Compound datatype handle
	herr_t		err;        // Function return status
	hsize_t 	n_doubles = sizeof(datalog_double)/sizeof(double);
	hsize_t 	n_floats = sizeof(datalog_float)/sizeof(float);
	hsize_t 	n_longs = sizeof(datalog_long)/sizeof(long);
	int 		n_fields = n_doubles + n_floats + n_longs;
	size_t 		offset[n_fields];
	hid_t 		field_type[n_fields];

	struct write_data{
		double Doubles[n_doubles];
		float Floats[n_floats];
		long Longs[n_longs];
	};

	struct write_data wBuffer; // Data struct to be written

	// Define offsets, field types, and field names
	const char *field_names[n_fields];
	int i, j, k;
	for (i=0; i < n_doubles; i++)
	{
		offset[i] = i * sizeof(double);
		field_names[i] = double_names[i];
		field_type[i] = H5T_NATIVE_DOUBLE;
	}
	for (j=0; j < n_floats; j++)
	{
		offset[i+j] = (i) * sizeof(double) + j * sizeof(float);
		field_names[i+j] = float_names[j];
		field_type[i+j] = H5T_NATIVE_FLOAT;
	}
	for (k=0; k < n_longs; k++)
	{
		offset[i+j+k] = (i) * sizeof(double) + (j) * sizeof(float) + k * sizeof(long);
		field_names[i+j+k] = long_names[k];
		field_type[i+j+k] = H5T_NATIVE_LONG;
	}

	// Create compound data type
	printf("Creating compound data type...\n\n");
	memtype = H5Tcreate(H5T_COMPOUND, sizeof(datalog_t));
	for (int i=0; i < n_fields; i++)
	{
		err = H5Tinsert(memtype, field_names[i], offset[i], field_type[i]);
		if(err < 0)
			goto out;
	}

	// Create .h5 file
	printf("Creating h5 file...\n\n");
	fid = H5Fcreate("packetData.h5",H5F_ACC_TRUNC,H5P_DEFAULT,H5P_DEFAULT);

	// Create packet table
	printf("Creating Packet Table...\n\n");
	ptable = H5PTcreate_fl(fid, "Packet Test Dataset", memtype, (hsize_t)5, 5);
	if(ptable == H5I_INVALID_HID)
		goto out;

	// Open binary file to read
	file = fopen(BIN_FILE, "rb");
	if (file == NULL)
	{
		printf("ERROR: Can't open testData.bin file!\n");
		exit(-1);
	}
	fseek(file, 0L, SEEK_END);
	long file_size = ftell(file);
	printf("File is %i bytes \n\n", file_size);
	fseek(file, 0L, SEEK_SET);

	// Append packet data
	printf("Appending packet data...\n\n");
	for( int p=0; p<file_size/sizeof(datalog_t); p++)
	{
		// read binary file
		fread(&wBuffer.Doubles, sizeof(double), n_doubles, file);
		fread(&wBuffer.Floats, sizeof(float), n_floats, file);
		fread(&wBuffer.Longs, sizeof(long), n_longs, file);

		err = H5PTappend(ptable, (hsize_t)1, &(wBuffer) ); //appends more packets to the end
		if(err < 0)
			goto out;
	}

	err = H5PTclose(ptable);
	if(err < 0)
		goto out;

	/* Close the file */
	H5Fclose(fid);
	return 0;

	out: /* An error has occurred.  Clean up and exit. */
		H5PTclose(ptable);
		H5Fclose(fid);
		return -1;
}
