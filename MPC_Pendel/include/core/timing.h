#ifndef timing_h
#define timing_h

#include <string>
#include <iostream>

#ifdef WIN32
#include <Windows.h>
#include <time.h>
#endif

#ifdef WIN32
#ifndef MINGW
#define DllExport __declspec( dllexport )
#pragma warning (disable : 4251)
#else
#define DllExport
#endif
#else 
#define DllExport
#endif


/** Zählen und Timen von Aufrufen */
class DllExport Timing {
public:
	Timing() : count(0), sumUser(0), sumReal(0) {} 
	void Start();
	void Stop();
	void Reset();
	static void GetTime(std::string &str);
	double beforeUser;
	int count;
	
#ifdef WIN32
	timeval beforeReal;
#else
	double beforeReal;
#endif
	double sumUser;
	double sumReal;
	
};

#define TIMING_CT 9
enum DllExport WORHPTiming_e {TIME_WORHP=0, TIME_OUTPUT, TIME_F, TIME_G, TIME_DF, TIME_DG, TIME_HM, TIME_FIDIF, TIME_ALL};


class DllExport WORHP_Timing {
	public:
		WORHP_Timing();
	
	Timing timing[TIMING_CT];
	
	void Start(WORHPTiming_e t) {timing[t].Start();}
	void Stop(WORHPTiming_e t) {timing[t].Stop();}
	
	

	void PrintTimes(std::ostream &os);
	void Bar(std::ostream &os, double z);
};


#endif
