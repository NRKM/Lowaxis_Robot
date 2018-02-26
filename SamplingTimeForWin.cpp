#include"SamplingTimeForWin.h"


void SAMPLING_TIME :: Initialize(double SetSamplingTime)
{
//	SetPriorityClass( GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
//	SetThreadPriority( GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
	timeBeginPeriod(1);
	SamplingTime = SetSamplingTime;
	TimeInitialize();
}


int SAMPLING_TIME :: WaitNextSamplingTime(double *time)
{
/*	QueryPerformanceCounter(&now);
	*time = (double)(now.QuadPart - old.QuadPart)/(double)freq.QuadPart;
	if(*time > SamplingTime)
	{
		old = now;
		return 0;
	}
	else return 1;
	*/
	//Sleep(1);
	QueryPerformanceCounter(&now);
	*time = (double)(now.QuadPart - old.QuadPart)/(double)freq.QuadPart;
	while(*time < SamplingTime)
	{
	 	QueryPerformanceCounter(&now);
		*time = (double)(now.QuadPart - old.QuadPart)/(double)freq.QuadPart;
	}
	old = now;
	return 1;
}


int SAMPLING_TIME :: GetTime(double *time)
{
	QueryPerformanceCounter(&now);
	*time = (double)(now.QuadPart - StartTime.QuadPart)/(double)freq.QuadPart;
	return 1;
}


void SAMPLING_TIME :: TimeInitialize()
{
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&StartTime);
	old = StartTime;
}


void SAMPLING_TIME :: end()
{
	timeEndPeriod(1);
}
