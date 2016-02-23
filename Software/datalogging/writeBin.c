/*
 * writeCsv.c
 *
 *  Created on: Feb 10, 2016
 *      Author: patipan
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "datalog.h"
#include <time.h>

int main()
{
	FILE *file;
	int i, j , k; // looping variables

	double doubleBuffer[150];
	float floatBuffer[25];
	long longBuffer[25];

	struct timespec tstart, tend; // Time variables
	double time_spent;

	// Create data arrays
	for(int x=0; x < 150; x++)
	{
		doubleBuffer[x] = rand()/(double)RAND_MAX * 50000;
	}
	for(int x=0; x < 25; x++)
	{
		floatBuffer[x] = rand()/(float)RAND_MAX * 1000;
		longBuffer[x] = rand();
	}
	printf("Double data is %i bytes\n", sizeof(doubleBuffer));
	printf("Float data is %i bytes\n", sizeof(floatBuffer));
	printf("Long data is %i bytes\n", sizeof(longBuffer));

	file = fopen("testData.bin", "wb");

	clock_gettime(CLOCK_REALTIME, &tstart);
	for (int p=0; p < 1000; p++)
	//while(1)
	{
		fwrite(doubleBuffer, sizeof(double), sizeof(doubleBuffer)/sizeof(double), file);
		fwrite(floatBuffer, sizeof(float), sizeof(floatBuffer)/sizeof(float), file);
		fwrite(longBuffer, sizeof(long), sizeof(longBuffer)/sizeof(long), file);
		fflush(file);
	}
	clock_gettime(CLOCK_REALTIME, &tend);

	fclose(file);

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
}
