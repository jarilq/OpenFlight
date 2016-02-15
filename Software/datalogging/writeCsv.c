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
	int i, j , k, x, p; // looping variables

	double doubleBuffer[150];
	float floatBuffer[25];
	long longBuffer[25];

	struct timespec tstart, tend; // Time variables
	double time_spent;

	// Create data arrays
	for(x=0; x < 150; x++)
	{
		doubleBuffer[x] = rand()/(double)RAND_MAX * 50000;
	}
	for(x=0; x < 25; x++)
	{
		floatBuffer[x] = rand()/(float)RAND_MAX * 50000;
		longBuffer[x] = rand();
	}

	file = fopen("testData.csv", "a");

	clock_gettime(CLOCK_REALTIME, &tstart);
	for (p=0; p < 1; p++)
	//while(1)
	{
		for (i=0; i < 150; i++)
		{
			fprintf(file,",%Lf", doubleBuffer[i]);
		}
		for (j=0; j < 25; j++)
		{
			fprintf(file,",%f", floatBuffer[j]);
		}
		for (k=0; k < 25; k++)
		{
			fprintf(file,",%li", floatBuffer[k]);
		}
		fprintf(file, "\n");
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
