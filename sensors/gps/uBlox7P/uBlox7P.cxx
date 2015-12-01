
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "uBlox7P_defs.hxx"
#include "../gpsInterface.hxx"
#include "../../../globaldefs.hxx"

#define GPS_PORT		"/dev/ttyO1"
#define GPS_BAUDRATE 	B115200
#define GPS_PACKET      NAV_PVT

int file;

// gps packet structure
GPS_PACKET gpsPacket;

void calcChecksum(unsigned char* CK);

bool processGPS();

int initGPS() {

	/* setting up UART */
    if((file = open(GPS_PORT, O_RDWR | O_NOCTTY | O_NONBLOCK))<0) {
        perror("Failed to open GPS port\n");
        return -1;
    }

    struct termios options;
    tcgetattr(file, &options);
    options.c_cflag = GPS_BAUDRATE | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    //options.c_cflag = GPS_BAUDRATE | CS8 | CREAD | CLOCAL;
   // options.c_iflag = IGNPAR | ICRNL;
    tcflush(file, TCIFLUSH);
    tcsetattr(file, TCSANOW, &options);
    fcntl(file,F_SETFL,O_NONBLOCK);
    return 0;
}

int readGPS(struct gps *gpsData_ptr) {
    // gps time of week
    static unsigned long prevTOW = 0;

    if( processGPS() ) {

        printf("DATA!\n");

        // assign GPS data
        gpsData_ptr->lat_rad    =   gpsPacket.lat/10000000.0 * D2R; 
        gpsData_ptr->lon_rad    =   gpsPacket.lon/10000000.0 * D2R;
        gpsData_ptr->alt_m      =   gpsPacket.hMSL/1000.0;
        gpsData_ptr->vn_mps     =   gpsPacket.velN/1000.0;
        gpsData_ptr->ve_mps     =   gpsPacket.velE/1000.0;
        gpsData_ptr->vd_mps     =   gpsPacket.velD/1000.0;
        gpsData_ptr->hAcc_m     =   gpsPacket.hAcc/1000.0;
        gpsData_ptr->vAcc_m     =   gpsPacket.vAcc/1000.0;
        gpsData_ptr->sAcc_mps   =   gpsPacket.sAcc/1000.0;
        gpsData_ptr->satVisible =   gpsPacket.numSV;

        // 3D-Fix
        if(gpsPacket.fixType == 0x03) {
            gpsData_ptr->valid  = 1;
        }
        else{
            gpsData_ptr->valid  = 0;
        }

        // GPS updated
        if((gpsPacket.iTOW - prevTOW) > 0) {
            gpsData_ptr->newData = 1;
            prevTOW = gpsPacket.iTOW;
        }
    }

    return 0;
}

int closeGPS() {

	// closing the UART
    close(file);

    return 0;
}

bool processGPS() {
    // UBLOX UBX header definition
    const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };

    // state of the parser
    static int fpos = 0;

    // checksum calculation
    static unsigned char checksum[2];

    // payload size
    const int payloadSize = sizeof(GPS_PACKET);

    // temporary buffer
    unsigned char c;

    // read buffer
    char buff[1];

    // read a byte from the serial port
    while ( (read(file,buff,1) > 0) ) {
        // cast to an unsigned char
        c = (unsigned char) buff[0];

        // identify the packet header
        if ( fpos < 2 ) {
            if ( c == UBX_HEADER[fpos] ) {
                fpos++;
            }
            else
                fpos = 0;
        }
        else {

            // grab the payload
            if ( (fpos-2) < payloadSize )
                ((unsigned char*)(&gpsPacket))[fpos-2] = c;
                fpos++;

            // compute checksum
            if ( fpos == (payloadSize+2) ) {
                calcChecksum(checksum);
            }
            else if ( fpos == (payloadSize+3) ) {
                if ( c != checksum[0] )
                    fpos = 0;
            }
            else if ( fpos == (payloadSize+4) ) {
                fpos = 0;
                if ( c == checksum[1] ) {
                    return true;
                }
            }
            else if ( fpos > (payloadSize+4) ) {
                fpos = 0;
            }
        }
    }
    return false;
}

// calculate the GPS data packet checksum
void calcChecksum(unsigned char* CK) {
    CK[0] = 0;
    CK[1] = 0;
    for (int i = 0; i < (int)sizeof(GPS_PACKET); i++) {
        CK[0] += ((unsigned char*)(&gpsPacket))[i];
        CK[1] += CK[0];
    }
}
