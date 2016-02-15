/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright by The HDF Group.                                               *
 * Copyright by the Board of Trustees of the University of Illinois.         *
 * All rights reserved.                                                      *
 *                                                                           *
 * This file is part of HDF5.  The full HDF5 copyright notice, including     *
 * terms governing use, modification, and redistribution, is contained in    *
 * the files COPYING and Copyright.html.  COPYING can be found at the root   *
 * of the source code distribution tree; Copyright.html can be found at the  *
 * root level of an installed copy of the electronic HDF5 document set and   *
 * is linked from the top-level documents page.  It can also be found at     *
 * http://hdfgroup.org/HDF5/doc/Copyright.html.  If you do not have          *
 * access to either file, you may request a copy from help@hdfgroup.org.     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "hdf5_hl.h"
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "datalog.h"
#include "createData.h"

/*-------------------------------------------------------------------------
 * Packet Table Fixed-Length Example
 *
 * Example program that creates a packet table and performs
 * writes and reads.
 *
 *-------------------------------------------------------------------------
 */

typedef struct{
	double Doubles[150];
	int Integers[25];
	long Longs[25];
} data_t;


int main(void)
{
	hid_t		fid;        /* File identifier */
	hid_t		ptable;     /* Packet table identifier */
	hid_t		array_tid;  /* Array datatype handle */
	hid_t		Int_array;
	hid_t		Long_array;
	hid_t		Double_array;
	hid_t		memtype;	/* Compound datatype handle */
	hid_t		strtype;	/* Variable length string type */

	herr_t		err;        /* Function return status */
	herr_t		status;		/* Datatype creation status */
	hsize_t		count;      /* Number of records in the table */
	hsize_t		array_dim[] = {5};	/* Array dimensions */
	hsize_t		Double_dim[] = {150};
	hsize_t		Int_dim[] = {25};
	hsize_t		Long_dim[] = {25};

	int			x;          /* Loop variable */
	int			p;
	int			i;
	int			j;
	int			k;

	double time_spent;		/* Timer variables */
	struct timespec tstart, tend;

	/* Buffers to hold data */
	datalog_double Buffer_double;
	datalog_float Buffer_float;
	datalog_long Buffer_long;
	double writeBuffer[5];

	/* Fill data */
	/*for(i=0; i<150; i++)
	{
		//wBuffer.Doubles[i] = rand()/(double)RAND_MAX * 50000;

	}
	for(j=0; j<25; j++)
	{
		//wBuffer.Integers[j] = rand() % 10000;
	}
	for(k=0; k<25; k++)
	{
		//wBuffer.Longs[k] = rand()/(double)RAND_MAX * 5000000;
	}*/
	//datalog_t wBuffer;
	datalog_t wBuffer = create_data();

	/* Create variable-length string datatype. */
	strtype = H5Tcopy(H5T_C_S1);
	status = H5Tset_size(strtype, H5T_VARIABLE);

//extern void init_datalog(){  //UNCOMMENT
	/* Initialize buffers */
	for(x=0; x<5; x++)
	{
		writeBuffer[x]=x*2.3;/*FOR FINAL VERSION replace this part so the buffer value is determined by the sensor readings*/
	}

	/* Create array data type */
	array_tid = H5Tarray_create(H5T_NATIVE_DOUBLE, 1, array_dim);
	/*Double_array = H5Tarray_create(H5T_NATIVE_DOUBLE,1, Double_dim);
	Float_array = H5Tarray_create(H5T_NATIVE_Float,1, Float_dim);
	Long_array = H5Tarray_create(H5T_NATIVE_LONG,1, Long_dim);*/

	/* Create compound data type */
	memtype = H5Tcreate(H5T_COMPOUND, sizeof(datalog_t));
	char str[50];
	int offset;
	printf("Creating Compound data type...\n\n");
	for (i=0; i < sizeof(datalog_double)/sizeof(double); i++)
	{
		offset = i * sizeof(double);
		snprintf(str,sizeof(str),"Doubles %d",i);
		status = H5Tinsert(memtype, str,offset, H5T_NATIVE_DOUBLE);
	}
	for (j=0; j < sizeof(datalog_float)/sizeof(float); j++)
	{
		offset = (i) * sizeof(double) + j * sizeof(float);
		snprintf(str,sizeof(str),"Floats %d",j);
		status = H5Tinsert(memtype, str, offset, H5T_NATIVE_FLOAT);
	}
	for (k=0; k < sizeof(datalog_long)/sizeof(long); k++)
	{
		offset = (i) * sizeof(double) + (j) * sizeof(float) + k * sizeof(long);
		snprintf(str,sizeof(str),"Longs %d",k);
		status = H5Tinsert(memtype, str, offset, H5T_NATIVE_LONG);
	}
    /* Create a file using default properties */
	printf("Creating h5 file...\n\n");
	fid = H5Fcreate("packetData.h5",H5F_ACC_TRUNC,H5P_DEFAULT,H5P_DEFAULT);

    /* Create a fixed-length packet table within the file */
    /* This table's "packets" will be array of doubles and it will use compression
     * level 5. */
	printf("Creating packet table...\n\n");
	ptable = H5PTcreate_fl(fid, "Packet Test Dataset", memtype, (hsize_t)500, 10);
	if(ptable == H5I_INVALID_HID)
		goto out;

//};  //UNCOMMENT

//extern void get_datalog(){};//UNCOMMENT

//extern void run_datalog() {//UNCOMMENT
	//err = H5PTclose(ptable);
	//H5Fclose(fid);
	printf("Appending packets to table...\n\n");
	clock_gettime(CLOCK_REALTIME, &tstart);
    /* Writes packets to the packet table */
	for(p=0; p<1; p++) //NOTE change the condition to run until the aircraft is done flying
	//while(1)
	{
		err = H5PTappend(ptable, (hsize_t)1, &(wBuffer) ); //appends more packets to the end
		//err = H5Fflush(fid, H5F_SCOPE_GLOBAL); // flush buffers of file to disk
		if(err < 0)
			goto out;
	}
	clock_gettime(CLOCK_REALTIME, &tend);

//};  //UNCOMMENT


//extern void close_datalog() { //UNCOMMENT
    /* Close the packet table */
	err = H5PTclose(ptable);
	if(err < 0)
		goto out;

    /* Close the file */
	printf("Closing file...\n\n");
	H5Fclose(fid);

	//time_spent = (double)(end-begin)/CLOCKS_PER_SEC;
	if ((tend.tv_nsec - tstart.tv_nsec) < 0)
	{
		time_spent = 1000000000 + tend.tv_nsec - tstart.tv_nsec;
	}
	else
	{
		time_spent = tend.tv_nsec - tstart.tv_nsec;
	}
	double time_sec = tend.tv_sec - tstart.tv_sec;
	printf("Execution time: %f seconds %f nanoseconds\n ", time_sec, time_spent);
	return 0;

	out: /* An error has occurred.  Clean up and exit. */
    	H5PTclose(ptable);
    	H5Fclose(fid);
    	return -1;
//}; //UNCOMMENT
}//delete when changing format?
