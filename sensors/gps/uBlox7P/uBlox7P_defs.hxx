
// UBLOX NAV PVT Data Structure
struct NAV_PVT {
  unsigned char   cls;
  unsigned char   id;
  unsigned short  len;
  unsigned long   iTOW;
  unsigned short  utcYear;
  unsigned char   utcMonth;
  unsigned char   utcDay;
  unsigned char   utcHour;
  unsigned char   utcMin;
  unsigned char   utcSec;
  unsigned char   valid;
  unsigned long   tAcc;
  long            utcNano;
  unsigned char   fixType;
  unsigned char   flags;
  unsigned char   reserved1;
  unsigned char   numSV;
  long            lon;
  long            lat;
  long            height;
  long            hMSL;
  unsigned long   hAcc;
  unsigned long   vAcc;
  long            velN;
  long            velE;
  long            velD;
  long            gSpeed;
  long            heading;
  unsigned long   sAcc;
  unsigned long   headingAcc;
  unsigned short  pDOP;
  unsigned short  reserved2;
  unsigned long   reserved3;
};
