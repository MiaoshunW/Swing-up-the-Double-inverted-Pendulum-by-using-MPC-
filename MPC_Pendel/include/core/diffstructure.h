#ifndef diffstructure_h
#define diffstructure_h

#include "../base/defines.h"
#include <iostream>
#include <map>

#include "../base/point.h"

class TransWorhp;


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


/** (dichtes) Abspeichern von Strukturinformationen und Ableitungswerten */
class DllExport DiffStructure {
	friend DllExport std::ostream &operator<<(std::ostream &os, const DiffStructure &d);
	friend class TransWorhp;
	friend class MagicTransWorhp;
	friend class ExplTransWorhp;
	friend class TWfolder;
	friend class Butcher;
	
public:
	/**
	 * Constructor
	 */
	DiffStructure();

	DiffStructure(const DiffStructure &other);
	/**
	 *
	 */
	~DiffStructure();

	/**
	 * Initialisierung
	 * @param eq Number of equations
	 * @param diff Number of dependency parameters
	 */
	void Init(int eq, int diff);
	
	/**
	 * Set sparsity structure (and set values)
	 * @param i equation index
	 * @param j dependency parameter index
	 */
	double& operator()(int i, int j);
	
	/**
	 * explTW only! Set sparsity structure for non-multinodes
	 */
	void operator()(int i, int j, int dis, int n_param);
	
	
	int activeindex;
	
	bool ode_power(int n_ode, int n_ctrl);
	bool neben_power(DiffStructure &DS_ode, int n_ode);

	bool isValid() const;


	// Fuer WORHP-Tool
	int GetEq() const;
	int GetDiff() const;
	int GetEntries() const;
	int CheckEntry(int i, int j) const;

private:
	
	struct DiffEntry {
		int use;
		double value[3];
	};
	
	int n_eq, n_diff;
	int adding;
	bool use_struct;
	std::map< Point<int>,  DiffEntry > entrymap;
	
	double get(int i, int j) const;
	
	void finish();

	int odecheck(int j, int l, int block_index, int block_eq, double &val, double &mult, DiscretizationType_e type);
	int check(int l, int j) const;
	
	double* find(int i, int j);
};

#endif
