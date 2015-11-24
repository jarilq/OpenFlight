/*
 * transfer.c
 *
 *  Created on: Nov 11, 2015
 *      Author: patipan
 */

#include<math.h>
#include<stdlib.h>

#include "transfer.hxx"

double tf(double input, double num[3], double den[3], double* RegX1, double* RegX2, double* RegY1, double* RegY2, double T)
{
	//			D*s^2 + E*s + F
	//	H(s) = -----------------
	//			A*s^2 + B*s + C

	// RegY[0] = y(k-1), RegY[1] = y(k-2), RegX[0] = x(k-1), RegX[1] = x(k-2)
	// T: time step

	double A, B, C; // denom coefficients
	double D, E, F; // num coefficients
	double a[3], b[3]; // discrete coefficients
	double output;

	// Coefficients
	A = den[0];
	B = den[1];
	C = den[2];
	D = num[0];
	E = num[1];
	F = num[2];

	//
	// y(k) = a[1]*y(k-1) + a[2]*y(k-2) + b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2)
	//

	// calculate discrete coefficients
	double div = A + T*B + T*T*C;
	a[1] = (2*A + T*B)/div;
	a[2] = (-A)/div;
	b[0] = (D + T*E + T*T*F)/div;
	b[1] = (-2*D - T*E)/div;
	b[2] = D/div;

	output = a[1]*(*RegY1) + a[2]*(*RegY2) + b[0]*input + b[1]*(*RegX1) + b[2]*(*RegX2);
	// set regression values
	*RegY2 = *RegY1;
	*RegY1 = output;
	*RegX2 = *RegX1;
	*RegX1 = input;

	return output;
}
