#ifndef newdouble_h
#define newdouble_h
#include "../base/defines.h"
#include <iostream>
#include <cmath>
#include <vector>

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

class MagicDouble;

struct MagicDeriv {
	MagicDeriv() : ref(0), diff(0) {}

	MagicDouble* ref;
	double diff;
	
};

struct Magic2Deriv {
	Magic2Deriv() : ref1(0), ref2(0), diff(0) {}

	MagicDouble* ref1;
	MagicDouble* ref2;
	double diff;
	
};

std::ostream &operator<<(std::ostream& os, const MagicDeriv& a);
std::ostream &operator<<(std::ostream& os, const Magic2Deriv& a);

/** Ableiten durch Code Überladen */
class DllExport MagicDouble {
	
private:
	MagicDouble();
	void depends_on(const std::vector<MagicDeriv> &other, double factor);
	void depends_also_on(const std::vector<MagicDeriv> &other, double factor);

	void depends2_on(const std::vector<Magic2Deriv> &other, double factor);
	void depends2_also_on(const std::vector<Magic2Deriv> &other, double factor);
	void depends2_also_on(const std::vector<MagicDeriv> &other, const std::vector<MagicDeriv> &other2, double factor);
	void scale(double factor);
	void scale2(double factor);
	
public:
	MagicDouble(double v);
	~MagicDouble() {
		depends.clear();
		depends2.clear();
	}
	void link();
	void unlink();
	
	/** Berechnen der 1. und 2. Ableitungen? */
	static int Derivate;
	
	static MagicDouble* NEW(int n);
	static MagicDouble* NO_DEP_NEW(int n);
	static void COPY(MagicDouble *ret, const double *data, int n);
	static void DEBUG_(MagicDouble *ret, int n);
	
	static int Stats[10];
	static void Info();
	
	/** Werte Zahl aus */
	double F() const {return val;}
	
	/** Werte 1. Ableitung aus */
	const double* DF(MagicDouble *d) const;
	
	/** Werte 2. Ableitung aus */
	const double* DDF(MagicDouble *d, MagicDouble *d2) const;

private:
	double val;
	std::vector<MagicDeriv> depends;
	std::vector<Magic2Deriv> depends2;

public:
	friend MagicDouble DllExport operator+(const MagicDouble& a, const MagicDouble& b);
	friend MagicDouble DllExport operator+(double a, const MagicDouble& b);
	friend MagicDouble DllExport operator+(const MagicDouble& a, double b);
	friend MagicDouble DllExport operator+(const MagicDouble& a);
	MagicDouble& operator+=(const MagicDouble& other);

	friend MagicDouble DllExport operator-(const MagicDouble& a, const MagicDouble& b);
	friend MagicDouble DllExport operator-(double a, const MagicDouble& b);
	friend MagicDouble DllExport operator-(const MagicDouble& a, double b);
	friend MagicDouble DllExport operator-(const MagicDouble& a);

	friend MagicDouble DllExport operator*(const MagicDouble& a, const MagicDouble& b);
	friend MagicDouble DllExport operator*(double a, const MagicDouble& b);
	friend MagicDouble DllExport operator*(const MagicDouble& a, double b);
	MagicDouble& operator*=(double other);

	friend MagicDouble DllExport operator/(const MagicDouble& a, const MagicDouble& b);
	friend MagicDouble DllExport operator/(double a, const MagicDouble& b);
	friend MagicDouble DllExport operator/(const MagicDouble& a, double b);

	
	friend MagicDouble DllExport sin(const MagicDouble& a);
	friend MagicDouble DllExport cos(const MagicDouble& a);
	friend MagicDouble DllExport exp(const MagicDouble& a);

	friend std::ostream DllExport &operator<<(std::ostream& os, const MagicDouble& a);

};

#endif
