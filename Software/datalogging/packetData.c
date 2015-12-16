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

/*-------------------------------------------------------------------------
 * Packet Table Fixed-Length Example
 *
 * Example program that creates a packet table and performs
 * writes and reads.
 *
 *-------------------------------------------------------------------------
 */

int main(void)
{
 hid_t          fid;        /* File identifier */
 hid_t          ptable;     /* Packet table identifier */

 herr_t         err;        /* Function return status */
 hsize_t        count;      /* Number of records in the table */

 int            x;          /* Loop variable */
 int 		p;

    /* Buffers to hold data */
 int writeBuffer[5];


//extern void init_datalog(){  //UNCOMMENT
   /* Initialize buffers */
 for(x=0; x<5; x++)
 {
     writeBuffer[x]=x*2;/*FOR FINAL VERSION replace this part so the buffer value is determined by the sensor readings*/
 }

    /* Create a file using default properties */
 fid=H5Fcreate("packetData.h5",H5F_ACC_TRUNC,H5P_DEFAULT,H5P_DEFAULT);

    /* Create a fixed-length packet table within the file */
    /* This table's "packets" will be simple integers and it will use compression
     * level 5. */
 ptable = H5PTcreate_fl(fid, "Packet Test Dataset", H5T_NATIVE_INT, (hsize_t)100, 5);
 if(ptable == H5I_INVALID_HID)
     goto out;
//};  //UNCOMMENT

//extern void get_datalog(){};//UNCOMMENT

//extern void run_datalog() {//UNCOMMENT
    /* Writes packets to the packet table */
for(p=0; p<5; p++) /*NOTE change the condition to run until the aircraft is done flying*/
{
 err = H5PTappend(ptable, (hsize_t)1, &(writeBuffer[p]) );//appends more packets to the end
 if(err < 0)
     goto out;
}
//};  //UNCOMMENT


//extern void close_datalog() { //UNCOMMENT
    /* Close the packet table */
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
//}; //UNCOMMENT
}//delete when changing format?
