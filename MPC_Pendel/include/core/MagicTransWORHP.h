#ifndef magictransworhp_h
#define magictransworhp_h

#include "../base/defines.h"

#include "TransWORHP.h"
#include "newdouble.h"

class DllExport MagicTransWorhp : public TransWorhp {

	friend class Viewer;
	//	friend std::ostream& operator<<(std::ostream &os, const TransWorhp &p);

public:
	/**
	* Base class for optimal control system.
	* @param s
	* @param dis
	* @param ode
	* @param ctrl
	* @param param
	* @param rand
	* @param neben
	*/
	MagicTransWorhp(const char *s, int dis, int ode, int ctrl, int param, int rand, int neben) ;
	virtual ~MagicTransWorhp();

	virtual void ode(MagicDouble *dx, double t, const MagicDouble *x, const MagicDouble *u,
		const MagicDouble *p)=0;

	void ode(double *dx, double t, const double *x, const double *u, const double *p);
	bool ode_structure(DiffStructure &s);

	void HermiteSimpson(double* G1, double *G2, int dis);
	void Trapez(double* ddx, int dis);
	void RechteSeite(double *G1, double t, int dis);

	void DG_diff_ode(double t, int dis, int active_index);
	void DG_diff_ode_p(double t, int l, int dis, int active_index);


	virtual void obj(MagicDouble *F, MagicDouble *X)=0;
	double obj();
	bool obj_structure(DiffStructure &s);
	//bool obj_diff(DiffStructure &s);
	void DF_calculate(WorhpMatrix &DF, double ScaleObj);


	int HM_structure(int hessianstructure, WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix *HM=0);
	void HM_calculate4(WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix &HM, double ScaleObj, double *Mu);
	void HM_calculate5(WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix &HM, double ScaleObj, double *Mu);

	void infoMagic();
	void localinit();


	void Objective(MagicDouble *F, double ScaleObj);
	void Constraints3(MagicDouble* XX, MagicDouble *GG);
	void Trapez(MagicDouble* XX, MagicDouble* ddx, int dis);
	void HermiteSimpson(MagicDouble* XX, MagicDouble* G1, MagicDouble *G2, int dis);

	void ParseXML(XMLNode *xmlmain);


private:
	MagicDouble *magic_X1;
	MagicDouble *magic_U1;
	MagicDouble *magic_P;

	MagicDouble *nomagic_DX1, *nomagic_DX12, *nomagic_DX2;
	MagicDouble *nomagic_X1, *nomagic_X12, *nomagic_X2;
	MagicDouble *nomagic_U1, *nomagic_U12, *nomagic_U2;

	MagicDouble *nomagic_P;

	MagicDouble *magic_XX;
	MagicDouble *nomagic_XX;
	MagicDouble *nomagic_GG;

	MagicDouble Lagrangian;


};


#endif
