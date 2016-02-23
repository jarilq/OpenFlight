/*
 * datalog.h
 *
 *  Created on: Feb 4, 2016
 *      Author: patipan
 */
#include <stdlib.h>

#ifndef DATALOGGING_DATALOG_H_
#define DATALOGGING_DATALOG_H_

typedef struct{
	double x1;
	double x2;
	double x3;
	double x4;
	double x5;
	double x6;
	double x7;
	double x8;
	double x9;
	double x10;
	double x11;
	double x12;
	double x13;
	double x14;
	double x15;
	double x16;
	double x17;
	double x18;
	double x19;
	double x20;
	double x21;
	double x22;
	double x23;
	double x24;
	double x25;
	double x26;
	double x27;
	double x28;
	double x29;
	double x30;
	double x31;
	double x32;
	double x33;
	double x34;
	double x35;
	double x36;
	double x37;
	double x38;
	double x39;
	double x40;
	double x41;
	double x42;
	double x43;
	double x44;
	double x45;
	double x46;
	double x47;
	double x48;
	double x49;
	double x50;
	double x51;
	double x52;
	double x53;
	double x54;
	double x55;
	double x56;
	double x57;
	double x58;
	double x59;
	double x60;
	double x61;
	double x62;
	double x63;
	double x64;
	double x65;
	double x66;
	double x67;
	double x68;
	double x69;
	double x70;
	double x71;
	double x72;
	double x73;
	double x74;
	double x75;
	double x76;
	double x77;
	double x78;
	double x79;
	double x80;
	double x81;
	double x82;
	double x83;
	double x84;
	double x85;
	double x86;
	double x87;
	double x88;
	double x89;
	double x90;
	double x91;
	double x92;
	double x93;
	double x94;
	double x95;
	double x96;
	double x97;
	double x98;
	double x99;
	double x100;
	double x101;
	double x102;
	double x103;
	double x104;
	double x105;
	double x106;
	double x107;
	double x108;
	double x109;
	double x110;
	double x111;
	double x112;
	double x113;
	double x114;
	double x115;
	double x116;
	double x117;
	double x118;
	double x119;
	double x120;
	double x121;
	double x122;
	double x123;
	double x124;
	double x125;
	double x126;
	double x127;
	double x128;
	double x129;
	double x130;
	double x131;
	double x132;
	double x133;
	double x134;
	double x135;
	double x136;
	double x137;
	double x138;
	double x139;
	double x140;
	double x141;
	double x142;
	double x143;
	double x144;
	double x145;
	double x146;
	double x147;
	double x148;
	double x149;
	double x150;
} datalog_double;

const char *double_names[150] = {"x1",		"x2",	"x3",	"x4",	"x5",
								 "x6",		"x7",	"x8",	"x9",	"x10",
								 "x11",		"x12",	"x13",	"x14",	"x15",
								 "x16",		"x17",	"x18",	"x19",	"x20",
								 "x21",		"x22",	"x23",	"x24",	"x25",
								 "x26",		"x27",	"x28",	"x29",	"x30",
								 "x31",		"x32",	"x33",	"x34",	"x35",
								 "x36",		"x37",	"x38",	"x39",	"x40",
								 "x41",		"x42",	"x43",	"x44",	"x45",
								 "x46",		"x47",	"x48",	"x49",	"x50",
								 "x51",		"x52",	"x53",	"x54",	"x55",
								 "x56",		"x57",	"x58",	"x59",	"x60",
								 "x61",		"x62",	"x63",	"x64",	"x65",
								 "x66",		"x67",	"x68",	"x69",	"x70",
								 "x71",		"x72",	"x73",	"x74",	"x75",
								 "x76",		"x77",	"x78",	"x79",	"x80",
								 "x81",		"x82",	"x83",	"x84",	"x85",
								 "x86",		"x87",	"x88",	"x89",	"x90",
								 "x91",		"x92",	"x93",	"x94",	"x95",
								 "x96",		"x97",	"x98",	"x99",	"x100",
								 "x101",	"x102",	"x103",	"x104",	"x105",
								 "x106",	"x107",	"x108",	"x109",	"x110",
								 "x111",	"x112",	"x113",	"x114",	"x115",
								 "x116",	"x117",	"x118",	"x119",	"x120",
								 "x121",	"x122",	"x123",	"x124",	"x125",
								 "x126",	"x127",	"x128",	"x129",	"x130",
								 "x131",	"x132",	"x133",	"x134",	"x135",
								 "x136",	"x137",	"x138",	"x139",	"x140",
								 "x141",	"x142",	"x143",	"x144",	"x145",
								 "x146",	"x147",	"x148",	"x149",	"x150"};

typedef struct{
	float x151;
	float x152;
	float x153;
	float x154;
	float x155;
	float x156;
	float x157;
	float x158;
	float x159;
	float x160;
	float x161;
	float x162;
	float x163;
	float x164;
	float x165;
	float x166;
	float x167;
	float x168;
	float x169;
	float x170;
	float x171;
	float x172;
	float x173;
	float x174;
	float x175;
} datalog_float;

const char *float_names[25] = {	"x151",		"x152",	"x153",	"x154",	"x155",
								"x156",		"x157",	"x158",	"x159",	"x160",
								"x161",		"x162",	"x163",	"x164",	"x165",
								"x166",		"x167",	"x168",	"x169",	"x170",
								"x171",		"x172",	"x173",	"x174",	"x175"};

typedef struct{
	long x176;
	long x177;
	long x178;
	long x179;
	long x180;
	long x181;
	long x182;
	long x183;
	long x184;
	long x185;
	long x186;
	long x187;
	long x188;
	long x189;
	long x190;
	long x191;
	long x192;
	long x193;
	long x194;
	long x195;
	long x196;
	long x197;
	long x198;
	long x199;
	long x200;
} datalog_long;

const char *long_names[25] = {	"x176",	"x177",	"x178",	"x179",	"x180",
								"x181",	"x182",	"x183",	"x184",	"x185",
								"x186",	"x187",	"x188",	"x189",	"x190",
								"x191",	"x192",	"x193",	"x194",	"x195",
								"x196",	"x197",	"x198",	"x199",	"x200"};

typedef struct{
	datalog_double Doubles;
	datalog_float Floats;
	datalog_long Longs;
} datalog_t;

#endif /* DATALOGGING_DATALOG_H_ */
