#ifndef expltransworhp_h
#define expltransworhp_h

#include "../base/defines.h"

#include "TransWORHP.h"

/** struct zum Speichern von Informationen ueber die die Opt.Variablen */
struct typeX {
	int type; // 0: Zustand, 1: Steuerung, 2: Parameter
	int n; // Nummer
	int multinodeL; // Multiknoten links
	int dis; // diskreter Punkt
};

class DllExport ExplTransWorhp : public TransWorhp {

	friend class Viewer;

public:
	/**
	* Base class for optimal control system (multi. shooting).
	* @param s Problem description
	* @param dis Number of discrete points
	* @param multinode vector of multinodes
	* @param ode Number of differential equations (or states)
	* @param ctrl Number of controls
	* @param param Number of free parameters (e.g. final time)
	* @param rand Number of non-trivial boundary constraints
	* @param neben Number of non-trivial control and state constraints
	* @param BOXneben Number of trivial state constraints (box constraints)
	* @param integral Number of lagrange terms
	* @param zen Number of non-trivial zen parameters
	*/
	ExplTransWorhp(const char *s, int dis, std::vector<int> &multinode, int ode, int ctrl, int param, int rand, int neben, std::vector<int> BOXneben=std::vector<int>(), int integral=0, int zen=0);
	
	/**
	* Base class for optimal control system (multi. shooting).
	* @param s Problem description
	* @param dis Number of discrete points
	* @param multi Nuber of multinodes (equidistant)
	* @param ode Number of differential equations (or states)
	* @param ctrl Number of controls
	* @param param Number of free parameters (e.g. final time)
	* @param rand Number of non-trivial boundary constraints
	* @param neben Number of non-trivial control and state constraints
	* @param BOXneben Number of trivial state constraints (box constraints)
	* @param integral Number of lagrange terms
	* @param zen Number of non-trivial zen parameters
	*/
	ExplTransWorhp(const char *s, int dis, int multi, int ode, int ctrl, int param, int rand=0, int neben=0, const std::vector<int> &BOXneben=std::vector<int>(), int integral=0, int zen=0);
	
	ExplTransWorhp(TWdimension &TWdata);
	
	/**
	*
	*/
	virtual ~ExplTransWorhp();
	
	
	void constInit();
	

	int DoubleFrom(TransWorhp *ph);

	/** Hochintegrieren */
	int Integrate(int btableau);
	

private:
	int integrate_schrittweite(int index, int DGflag=0);
//#ifdef _OPENMP
	int integrate_schrittweite_parallel(int multi, int index, int DGflag=0);

	/** Blockweise tmp-Speicher fuer parallele Integration */
	double *tmp_ode_parallel;
	/** Blockweise tmp-Speicher fuer parallele Integration */
	double *tmp_ctrl_1_parallel;
	/** Blockweise tmp-Speicher fuer parallele Integration */
	double *tmp_ctrl_2_parallel;
	
	/** Anzahl der Threads */
	int n_threads;
	
	/** Butcher-Objekte fuer parallele Integration */
	std::vector<Butcher*> butcher_parallel;
//#endif
	std::vector<std::vector<double> > schrittweiten;

	
public:
	/** initialisiert das Butcher-Tableau.
	* @param index Tableau-Nummer
	* @param stepsize Schrittweite
	*/
	void butcherInit(int index, double stepsize=1e-3, bool verbose=false);
	
	void infoMagic() {}


	/** Zugriffsfunktionen */
	double x(int dis, int ode) const override;
	double u(int dis, int ctrl) const override;
	double p(int param) const override;

	/** Varianten fuer Zugriff */
	double x__(int dis, int ode) const override;
	double u__(int dis, int ctrl) const override;

	/** Indexbestimmung */
	int x_index(int dis, int ode) const override;
	int x_index__(int dis, int ode) const override;
	int u_index(int dis, int ctrl) const override;
	int u_index__(int dis, int ode) const override;
	int p_index(int param) const override;
	
	/** bestimmt die Anzahl der abhaengigen Variablen fuer x_index 
	 *(fuer Knoten die keine Multiknoten sind)
	 */
	int aux_x_index(int dis, int ode);

	/** laeufte ueber die Stuetzstellen, ausser die Multi-Knoten */
	int dis_index(int dis);
private:
	/** Knotenindezies ohne Multiknoten */
	std::vector<int> T_ohneMulti;
	/** speichert, ob Knoten ein Multiknoten ist.
	* nicht direkt drauf zugreifen, sondern ueber "isMultinote()"
	*/
	std::vector<int> isMulti;
	
	/** speichert den Typ, die Nummer, dis. Punkt, linken Multiknoten der Opt.Varaiblen.
	* wird in init0() gefuellt
	*/
	std::vector<typeX> typeOfX;
public:
	/** @return 1, wenn Multiknoten, sonst 0 */
	int isMultinode(int i);
	
	int x_indexode(int ode) const override;
	int u_indexode(int ctrl) const override;
	int p_indexode(int param) const override;


	/** Implementierung des OCP */

	///** Zielfunktion */
	//double obj() = 0;
	//bool obj_structure(DiffStructure &s) {return false;}
	//bool obj_diff(DiffStructure &s) {return false;}

	//void integral(double *f, double t, const double *x, const double *u, const double *p) {}
	//bool integral_structure(DiffStructure &s) {return false;}
	//bool integral_diff(DiffStructure &s, double t, const double *x, const double *u, const double *p) {return false;}

	//void Lagrange();

	///** ODE-System */
	//void ode(double *dx, double t, const double *x, const double *u, const double *p)=0;

	//bool ode_structure(DiffStructure &s) {return false;}
	//bool ode_diff(DiffStructure &s, double t, const double *x, const double *u, const double *p) {return false;}
	//bool ode_diff_p(DiffStructure &s, double t, const double *x, const double *u, const double *p, int index) {return false;}
	virtual bool ode_diff2(DiffStructure &ds, double t, const double *x, const double *u, const double *p);

	///** Box-Beschraenkungen */
	//void x_boundary(double *x_low, double *x_upp) {}
	//void u_boundary(double *u_low, double *u_upp) {}
	//void p_boundary(double *p_low, double *p_upp) {}
	//void var_boundary(double *x_low, double *x_upp) {}

	///** Rand-Bedingungen */
	//void rand(double *r) {};
	//void rand_boundary(double *r_low, double *r_upp) {}
	//bool rand_structure(DiffStructure &s) {return false;}
	//bool rand_diff(DiffStructure &s) {return false;}

	///** Nebenbedingungen */
	//void neben(double *c, double t, const double *x, const double *u, const double *p) {}
	//void neben_boundary(double *c_low, double *c_upp) {}
	//bool neben_structure(DiffStructure &s) {return false;}
	//bool neben_diff(DiffStructure &s, double t, const double *x, const double *u, const double *p) {return false;}
	//bool neben_diff_p(DiffStructure &s, double t, const double *x, const double *u, const double *p, int index) {return false;}

	///** Startschaetzung */
	//void init() {}
	//void p_init(double *p) {};
	//void x_init(double *x, int i, int dis) {}
	//void u_init(double *u, int i, int dis) {}
	//
	//void zen_init(double *zen) {};

	///** Text-Ausgabe */
	//void terminate() {}
	//int step() {return true;}

	/** Grafische Ausgabe */
//#ifndef NOGRAPHICS
//	void OpenWindows(Viewer *gr) {};
//	void GetXTitle(int d, char *s) {s[0] = 0;}
//	void GetUTitle(int d, char *s) {s[0] = 0;}
//	//virtual std::string GetXTitle(int d) {return "";}
//	//virtual std::string GetUTitle(int d) {return "";}
//#endif


	//void TimeAxis(double exponent);

	void GetState(double *x, double t) override;
	void GetControl(double *uu, double t) override;

	void GetBoundaryIndices(std::vector<int> &indices, int d) override;
protected:
	virtual double &setx(int dis, int ode) override;

public:
	/** Speicher und Struktur */
	void GetSpace(int delta1, int delta2) override;
	/** Verbindung zu WORHP */
	void Connect(const OptVar &o, const Params &p) override;
	/** Beschraenkungsschranken */
	void Boundary() override;
	/** Beschraenkungen */
	void Constraints() override;
	/** Beschraenkungen mit anderem G Vektor */
	void Constraints2(double *GG, int DGflag=0) override;


protected:
	void DG_diff_ode(double t, int k, int active_index) override;
	void DG_diff_ode_p(double t, int l, int dis, int active_index) override;

	void DG_diff_rand(int k) override;

	void DG_diff_neben(double t, int dis) override;
	void DG_diff_neben_p(double t, int l, int dis) override;

public:
	double Objective(double ScaleObj) override;

	int DF_structure(WorhpMatrix *DF=0, int offset=0) override;
	void DF_calculate(WorhpMatrix &DF, double ScaleObj) override;

	int DG_structure(const TWfolder *f, WorhpMatrix *DG=0, int offset=0) override;
	void DG_calculate(TWfolder *f, WorhpMatrix &DG) override;

	int HM_structure(int, WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix *HM=0, int offset=0) override;
	int HM_structure_ohne_Diag(int, WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix *HM=0, int offset=0) override;
	void HM_calculate(int hessianvalues, WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix &HM, double ScaleObj, double *Mu) override;

	void Lagrange() override;

	void init0() override;

	/** Mehrzielknoten. */
	std::vector<int> multinode;

	/* Zugriffsmethoden */
	/** Zugriff auf Zustaende.
	* speichert auch temp. Zustaende, die keine Optimierungsvariablen sind
	*/
	double **pointer_x;
	double **pointer_u;
	double *pointer_p;

protected:

	/** Stetigkeitsbedingung fuer Mehrfachschiessen. */
	void Continuous(double* ddx, int dis);

	double *block_x;
	double *block_u;

	/** Hochintegrieren der Zustaende */
	void IntegrateStates();
	/** Hochintegrieren der Zustaende (parallel) */
	void IntegrateStates_parallel();
	/** Hochintegrieren der Zustaende in Intervall [start,end] */
	void IntegrateStates2(int start, int end, int DGflag=0);
	/** Hochintegrieren der Zustaende als Startschaetzung.
	* bis auf das setzen der Werte an den Multiknoten identisch zu "IntegrateStates"
	* @return Anzahl der Integrationsschritte
	*/
	int IntegrateInitial();

private:
	/** beschraenkte Zustaende */
	const std::vector<int> boxNB;

public:
	/** Anzahl der Zustandesbeschraenkungen */
	int n_boxNeben;

private:
	/* Methoden zum Erweitern der Box-Schranken auf Nebenbedingungen
	fuer Zustande die keine Optimierungsvariable sind */
	void boxNeben(double *c, const double *x);
	void boxNeben_boundary(double *c_low, double *c_upp);
};

#endif
