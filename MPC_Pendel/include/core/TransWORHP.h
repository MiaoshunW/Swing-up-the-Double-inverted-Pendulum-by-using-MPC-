#ifndef transworhp_h
#define transworhp_h

#include "../base/defines.h"

#include <iostream>
#include <iomanip>
#include <sstream>

#include "xmlio.h"
#include "worhp/worhp.h"

#include "diffstructure.h"
#include "butcher.h"

#include "TWparameter.h"
#include "TWcount.h"
#include "TWfolder.h"

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

class TWbaseSpline;

#ifndef NOGRAPHICS
#include "Viewer.h"
#endif


//typedef void (*ext_io_func) (int mode, const char *s);

//void DllExport MyStatus(const std::string &tag, const std::string &s, int flag);
//void DllExport MyOutputFunction(ext_io_func f);
//void DllExport SetMyStatusStream(std::ostream &os);

class TWfolder;

/**
* struct fuer (internen) Export von TW Daten
* @author Matthias Rick
*/
struct exportTW {
	std::vector<double> T; // Zeit
	std::vector<double> p; // Parameter
	std::vector<std::vector<double> > x; // Zustaende
	std::vector<std::vector<double> > u; // Steuerung
	std::vector<std::vector<double> > La; // Lambda
	std::vector<std::vector<double> > Mu; // Mu
	TWdiscretization *twdiscretisation; // Diskretisierung
	bool ok; // alles richtig gesetzt?
};

struct TWdimension {
	
	const char *ID;
	int n_dis;
	int n_ode;
	int n_ctrl;
	int n_param;
	int n_rand;
	int n_neben;
	int n_integral;
	int n_zen;
	//only for explTW
	std::vector<int> multinode;
	std::vector<int> BOXneben;
	
	TWdimension() : ID(nullptr), n_dis(2),
			n_ode(0), n_ctrl(0), n_param(0),
			n_rand(0), n_neben(0),
			n_integral(0), n_zen(0) {
			
		BOXneben = std::vector<int>();
	}
};

enum TransWORHP_type {fullDiscretization=0, multipleShooting=2};

/** TransWorhp */
class DllExport TransWorhp {

	friend class Viewer;

public:
	/**
	* Base class for optimal control system.
	* @param s Problem description
	* @param dis Number of discrete points
	* @param ode Number of differential equations (or states)
	* @param ctrl Number of controls
	* @param param Number of free parameters (e.g. final time)
	* @param rand Number of non-trivial boundary constraints
	* @param neben Number of non-trivial control and state constraints
	* @param integral Number of lagrange terms
	* @param zen Number of non-trivial zen parameters
	*/
	TransWorhp(const char *s, int dis, int ode, int ctrl, int param, int rand, int neben, int integral=0, int zen=0);
	
	TransWorhp(TWdimension &TWdata);

	/**
	* Destruktor
	*/
	virtual ~TransWorhp();
	
	static void MatrixToMATLAB(const WorhpMatrix &m, const std::string& filename);
	static void DoubleToMATLAB(double *data, int n, const std::string& filename);


	virtual int DoubleFrom(TransWorhp *ph);

	/**
	* exportiert Zustand und Steuerung in m-Datei.
	* @param filename Dateiname
	*/
	virtual void ToMATLAB(const std::string& filename);
	/**
	* importiert Zustand und Steuerung aus m-Datei.
	* @param filename Dateiname
	*/
	virtual void FromMATLAB(const std::string& filename);
	/**
	* exportiert Lambda in m-Datei.
	* @param filename Dateiname
	*/
	virtual void ToMATLAB_LambdaMu(const std::string& filename); //Matthias Rick
	/**
	* importiert Lambda aus m-Datei.
	* @param filename Dateiname
	*/
	virtual void FromMATLAB_LambdaMu(const std::string& filename); //Matthias Rick

	virtual int Integrate(int btableau);
	virtual int integrate(int index); //TODO besserer Name? verwirrrt doch arg.
	virtual int integrateRKF(int index, int &startflag);


	virtual void Debug_G();
	virtual void Debug_G2();

	/**
	* gibt an, welche Beschraenkung an der Stelle i ist (ODE,RAND,NB,etc).
	* @param Kompomente des G Vektor
	* @return Typ, zB ODE 0, RAND 1, etc
	*/
	std::string type_G(int i);

	friend std::ostream& operator<<(std::ostream &os, const TransWorhp &p);

	void Structure_Sizes(TWfolder *f, int hessianvalues, int &DF_nnz, int &DG_nnz, int &HM_nnz);
	void LinearTimeAxis(double start, double end);

	virtual void infoMagic();


	/* --------------- Zugriffsfunktionen --------------- */
	/** gibt den Zustand an Gitterpunkt zurueck.
	* @param dis dis. Punkt
	* @param ode Zustand
	* @return Zustand 'ode' an Stelle 'dis'
	*/
	virtual double x(int dis, int ode) const;
	/** gibt die Steuerung an Gitterpunkt zurueck.
	* @param dis dis. Punkt
	* @param ctrl Steuerung
	* @return Steuerung 'ctrl' an Stelle 'dis'
	*/
	virtual double u(int dis, int ctrl) const;
	/** gibt freien Parameter zurueck.
	* @param param freier Parameter
	* @return freier Parameter
	*/
	virtual double p(int param) const;

	/* --------------- Varianten fuer Zugriff --------------- */
	/** gibt den Zustand an Gitterpunkt zurueck.
	* gibt den Zustand an Gitterpunkt zurueck, wobei auch Zwischenpunkte
	* beachtet werden (zB bei HermiteSimpson)
	* @param dis dis. Punkt
	* @param ode Zustand
	* @return Zustand 'ode' an Stelle 'dis'
	*/
	virtual double x__(int dis, int ode) const;
	/** gibt die Steuerung an Gitterpunkt zurueck.
	* gibt die Steuerung an Gitterpunkt zurueck, wobei auch Zwischenpunkte
	* beachtet werden (zB bei HermiteSimpson)
	* @param dis dis. Punkt
	* @param ctrl Steuerung
	* @return Steuerung 'ctrl' an Stelle 'dis'
	*/
	virtual double u__(int dis, int ctrl) const;

	/* --------------- Indexbestimmung --------------- */
	/** gibt den Index im Optimierungsvektor zurueck.
	* @param dis dis. Punkt
	* @param ode Zustand
	* @return Index
	*/
	virtual int x_index(int dis, int ode) const;
	/** gibt den Index im Optimierungsvektor zurueck.
	* es werden auch Zwischenpunkte beruecksichtigt
	* @param dis dis. Punkt
	* @param ode Zustand
	* @return Index
	*/
	virtual int x_index__(int dis, int ode) const;
	/** gibt den Index im Optimierungsvektor zurueck.
	* @param dis dis. Punkt
	* @param ctrl Steuerung
	* @return Index
	*/
	virtual int u_index(int dis, int ctrl) const;
	/** gibt den Index im Optimierungsvektor zurueck.
	* es werden auch Zwischenpunkte beruecksichtigt
	* @param dis dis. Punkt
	* @param ctrl Steuerung
	* @return Index
	*/
	virtual int u_index__(int dis, int ctrl) const;
	/** gibt den Index im Optimierungsvektor zurueck.
	* @param param freier Parameter
	* @return Index
	*/
	virtual int p_index(int param) const;

	/** gibt den Index des Zustandes zureuck.
	* @param ode Zustand
	* @return Index
	*/
	virtual int x_indexode(int ode) const;
	/** gibt den Index der Steuerung zureuck.
	* @param ctrl Steuerung
	* @return Index
	*/
	virtual int u_indexode(int ctrl) const;
	/** gibt den Index der freien Parameter zureuck.
	* @param param freier Paramter
	* @return Index
	*/
	virtual int p_indexode(int param) const;


	/* --------------- Implementierung des OCP --------------- */

	/** Zielfunktion.
	* @return Zielfunktionswert
	*/
	virtual double obj()=0;
	/** Struktur der Zielfunktion.
	* optional
	* @param s Abhaengigkeit
	* @return true, wenn Struktur benutzt werden soll
	*/
	virtual bool obj_structure(DiffStructure &s);
	/** Ableitung der Zielfunktion.
	* optional
	* @param s Abhaengigkeit
	* @return true, wenn Ableitung benutzt werden soll
	*/
	virtual bool obj_diff(DiffStructure &s);

	virtual void integral(double *f, double t, const double *x, const double *u, const double *p);
	virtual bool integral_structure(DiffStructure &s);
	virtual bool integral_diff(DiffStructure &s, double t, const double *x, const double *u, const double *p);

	/** ODE-System / Dynamik */
	virtual void ode(double *dx, double t, const double *x, const double *u, const double *p)=0;
	/** Struktur des ODE-System */
	virtual bool ode_structure(DiffStructure &s);
	/** Ableitung des ODE-System */
	virtual bool ode_diff(DiffStructure &s, double t, const double *x, const double *u, const double *p);
	/** Ableitung des ODE-System nach freien Parameter */
	virtual bool ode_diff_p(DiffStructure &s, double t, const double *x, const double *u, const double *p, int index);
	
	/** Box-Beschraenkungen fuer Zustand */
	virtual void x_boundary(double *x_low, double *x_upp);
	/** Box-Beschraenkungen fuer Steuerung */
	virtual void u_boundary(double *u_low, double *u_upp);
	/** Box-Beschraenkungen fuer freie Parameter */
	virtual void p_boundary(double *p_low, double *p_upp);
	/** allgemeine Box-Beschraenkungen */
	virtual void var_boundary(double *x_low, double *x_upp);

	/** Randbedingungen */
	virtual void rand(double *r);
	/** Schranken fuer Randbedingungen */
	virtual void rand_boundary(double *r_low, double *r_upp);
	/** Struktur der Randbedingungen */
	virtual bool rand_structure(DiffStructure &s);
	/** Ableitung der Randbedingungen */
	virtual bool rand_diff(DiffStructure &s);

	/** Nebenbedingungen */
	virtual void neben(double *c, double t, const double *x, const double *u, const double *p);
	/** Schranken fuer Nebenbedingungen */
	virtual void neben_boundary(double *c_low, double *c_upp);
	/** Struktur der Nebenbedingungen */
	virtual bool neben_structure(DiffStructure &s);
	/** Ableitung der Nebenbedingungen */
	virtual bool neben_diff(DiffStructure &s, double t, const double *x, const double *u, const double *p);
	/** Ableitung der Nebenbedingungen nach freien Parametern */
	virtual bool neben_diff_p(DiffStructure &s, double t, const double *x, const double *u, const double *p, int index);

	/* --------------- Startschaetzung --------------- */
	/** wird vor der Optimierung einmalig aufgerufen */
	virtual void init();

	/** Initialisierung der Zustaende
	* @param x Zustand
	* @param i dis Punkt
	* @param dis Anz aller Gitterpunkte
	*/
	virtual void x_init(double *x, int i, int dis);
	/** Initialisierung der Steuerungen
	* @param u Steuerung
	* @param i dis Punkt
	* @param dis Anz aller Gitterpunkte
	*/
	virtual void u_init(double *u, int i, int dis);
	/** Initialisierung der freien Parameter
	* @param p freie Parameter
	*/
	virtual void p_init(double *p);

	virtual void zen_init(double *zen);

	/* --------------- Text-Ausgabe --------------- */
	/** wird nach Optimierung aufgerufen */
	virtual void terminate();
	/** wird nach jeden Optimierungsschritt aufgerufen */
	virtual int step();

	/** Grafische Ausgabe */
#ifndef NOGRAPHICS
	virtual void OpenWindows(Viewer *gr);
	virtual std::string GetXTitle(int d);
	virtual std::string GetUTitle(int d);
	/** Methode zur Auswahl welche Fenster dargestellt werden sollen */
#endif
	virtual void selectWindows(Viewer *viewer);

	void TimeAxis(double exponent);

	virtual void GetState(double *x, double t);
	virtual void GetControl(double *uu, double t);

	//void PrintNLPconstraints(ostream *os=0);
	void PrintOCPstates(std::ostream *os=0);
	void PrintMultipliers(std::ostream *os=0);

protected:
	virtual double &setx(int dis, int ode);
public:
	/** Speicher und Struktur */
	virtual void GetSpace(int delta1, int delta2);
	/** Verbindung zu WORHP */
	virtual void Connect(const OptVar &o, const Params &p);
	/** Beschraenkungsschranken */
	virtual void Boundary();
	/** Beschraenkungen */
	virtual void Constraints();
	/** Beschraenkungen mit anderem G Vektor */
	virtual void Constraints2(double *GG, int DGflag=0);


protected:
	virtual void DG_diff_ode(double t, int k, int active_index);
	virtual void DG_diff_ode_p(double t, int l, int dis, int active_index);

	virtual void DG_diff_rand(int k);

	virtual void DG_diff_neben(double t, int dis);
	virtual void DG_diff_neben_p(double t, int l, int dis);

public:
	virtual double Objective(double ScaleObj);

	virtual int DF_structure(WorhpMatrix *DF=0, int offset=0);
	virtual void DF_calculate(WorhpMatrix &DF, double ScaleObj);

	virtual int DG_structure(const TWfolder *f, WorhpMatrix *DG=0, int offset=0);
	virtual void DG_calculate(TWfolder *f, WorhpMatrix &DG);

	virtual int HM_structure(int, WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix *HM=0, int offset=0);
	virtual int HM_structure_ohne_Diag(int, WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix *HM=0, int offset=0);
	virtual void HM_calculate(int hessianvalues, WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix &HM, double ScaleObj, double *Mu);
	virtual void HM_calculate1(WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix &HM, double ScaleObj, double *Mu);
	virtual void HM_calculate2(WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix &HM, double ScaleObj, double *Mu);
	virtual void HM_calculate3(WorhpMatrix &HM, double ScaleObj);
	virtual void HM_calculate4(WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix &HM, double ScaleObj, double *Mu);
	virtual void HM_calculate5(WorhpMatrix &DF, WorhpMatrix &DG, WorhpMatrix &HM, double ScaleObj, double *Mu);

	virtual void Lagrange();

	virtual void init0();

	virtual void GetBoundaryIndices(std::vector<int> &indices, int d);

protected:
	virtual void localinit();

	int getncon();
	int getnvar();

	void tempmatrixinit();

	virtual void ParseXML(XMLNode *xmlmain);

	virtual void HermiteSimpson(double* G1, double *G2, int dis);
	virtual void Lobatto(double* G1, double *G2, double *G3, int dis);
	virtual void Trapez(double* ddx, int dis);
	virtual void Euler(double* ddx, int dis);
	virtual void RechteSeite(double *G1, double t, int dis);


public:

	std::string id; /**< Problemname */
	int unique_id;

	int n_dis; /**< Anzahl Gitterpunkte */
	const int n_ode; /**< Anzahl Zustaende */
	const int n_ctrl; /**< Anzahl Steuerungen */
	const int n_param; /**< Anzahl freier Parameter */
	const int n_rand; /**< Anzahl Randbedingungen */
	const int n_neben; /**< Anzahl Nebenbedingungen */
	const int n_integral; /**< Anzahl Integralterm */

	int Delta1; /**< Offset fuer WORHP X */
	int Delta2; /**< Offset fuer WORHP G */
	int n_var; /**< Anzahl Optimierungsvariablen */
	int n_con; /**< Anzahl Beschraenkungen */
	int n_zen;

	double *T; /**< Zeit-Diskretisierung */
	double *X; /**< Optimierungsvektor */
	double *G; /**< Beschraenkungen */
	double *ZEN; // Zen
	
	double *X_low, *X_upp;
	double *G_low, *G_upp;

	double *Lambda, *Mu;

	bool freetime; /**< freie Endzeit */

	Viewer *viewer; /**< Ausgabefenster */

	int Interrupt;

	TWparameter *twparameter;

	Butcher *butcher; /**< Integrationsverfahren */

	/** Info ueber aktuelles Diskretisierungsschema */
	TWdiscretization *twdiscretization;

	/** Loesungsverfahren.
	* 0 = direktes (normales) Verfahren
	* 2 = Mehrzielmethode
	*/
	TransWORHP_type transworhp_type;

	double *lagrange_integral;
	double *lagrange_weight;

	TWcount_calls twcount_calls;

	int DF_start, DF_nnz;
	int DG_start, DG_nnz;
	int HM_start, HM_start_diag, HM_nnz_ohne_diag;

	double t0; /**< Startzeitpunkt */

	int n_multinodes; /**< Anzahl der Mehrzielknoten (explTW) */

	double Infty; /**< WORHP-Unendlich */ //public: damit es von aussen ueberpruefbar ist

	double *tmp_gg_1, *tmp_gg_2, *tmp_gg_3, *tmp_gg_4, *tmp_gg_5;

	WorhpMatrix DFtemp1,DGtemp1,DFtemp2,DGtemp2;
	int tempmatrixinitialised;

protected:
	int SHOWDF, SHOWDG, SHOWHM;

	double eps;

	/** WORHP Data */
	std::string paramfile_worhp;
	int USERDF, USERDG, USERHM;

	// Hilfsvariablen fuer Auswertung ODE, NB, etc.
	double *tmp_ode_1, *tmp_ode_2;
	double *tmp_ctrl_1, *tmp_ctrl_2;
	double *tmp_rand_1, *tmp_rand_2;
	double *tmp_neben_1, *tmp_neben_2;
	double *tmp_integral_1, *tmp_integral_2, *tmp_integral_12;

	DiffStructure DS_obj, DS_ode, DS_neben, DS_rand, DS_integral;


/*Methoden fuer Dis.fehler - Matthias Rick*/
private:
	/**
	* Wertet die Differenz von Dynamik und Spline an Punkt t aus (vgl. Betts)
	* -> Hilfsfunktion fuer disFehlerIntegral
	* @author Matthias Rick
	* @param t Zeitpunkt
	* @param komp Komponente
	* @param zustand Zustand
	* @param steuerung Steuerung
	* @return |Dynamik - Spline|
	*/
	inline double auxFehlerInt(double t, int komp, TWbaseSpline **zustand, TWbaseSpline **steuerung);
	/**
	* Berechnet Integral ( |spline(x)' - f(spline(x),spline(u),t)| )
	* -> Hilfsfunktion fuer diskretisierungsfehlerBetts
	* @author Matthias Rick
	* @param fehler Rueckgabewert: Fehler pro Intervall
	* @param a Intervall Anfang
	* @param b Intervall Ende
	* @param zustand Zustand
	* @param steuerung Steuerung
	*/
	void disFehlerIntegral(double *fehler,const double a, const double b, TWbaseSpline **zustand, TWbaseSpline **steuerung);
	/**
	* berechnet den Diskretisierungsfehler nach Betts
	* @author Matthias Rick
	*/
	void diskretisierungsfehlerBetts();
	/**
	* berechnet den Diskretisierungsfehler durch Vergleich mit Verfahren hoeherer Ordnung
	* Euler mit Trapez
	* Trapez mit Hermite-Simpson
	* @author Matthias Rick
	*/
	void diskretisierungsfehler2();
public:
	/** Diskretisierungsfehler */
	std::vector<std::vector<double> > FEHLER;
	/** Diskretisierung */
	std::vector<std::vector<double> > SCHRITTWEITE;

	/**
	* Berechnung des Diskretisierungsfehler.
	* berechnet den Diskretisierungsfehler und schreibt ihn in FEHLER
	* @author Matthias Rick
	*/
	void diskretisierungsfehler();

	/**
	* Bestimmt das neue Gitter
	* @author Matthias Rick
	* @param max aktuelles Maximum (groesster Fehler)
	* @return Vektor mit neuer Diskretisierung
	*/
	std::vector<double> refineAlg(double &max);
private:
	/**
	* Berechnet das neue Gitter nach Betts
	* @author Matthias Rick
	* @param I Vektor in den die Anzahl an neuen Punkten pro Intervall geschrieben wird
	* @param max aktuelles Maximum
	*/
	void betts(int *I, double &max);
	/**
	* Berechnet Auf-/Absprungpunkte nach Bueskens
	* @author Matthias Rick
	* @return Vektor mit neuen Punkten
	*/
	std::vector<double> bueskens();

	/**
	* gibt das aktuelle Gitter als Vektor zurueck
	* @author Matthias Rick
	* @return Gitterpunkte als Vektor
	*/
public:
	std::vector<double> getGrid();
	/**
	* setzt das Zeitgitter
	* @author Matthias Rick
	* @return Gitterpunkte als Vektor
	*/
	void newGrid(std::vector<double> grid);

	/**
	* exportieren von TW Daten
	* Zeit, Param, Zustand, Steuer, Lambda, Mu
	* @author Matthias Rick
	* @return exportTW-Struct
	*/
	exportTW outTW();

	/**
	* importieren von TW Daten
	* Zeit, Param, Zustand, Steuer, Lambda, Mu
	* @author Matthias Rick
	* @param eTW TW-Struct
	*/
	void inTW(exportTW eTW);
};






class DllExport ScaleTransWorhp : public TransWorhp {

public:

	ScaleTransWorhp(const char *s, int dis, int ode, int ctrl, int param, int rand, int neben, int integral=0) :
	  TransWorhp(s,dis,ode,ctrl,param,rand,neben,integral) {

	  }

	  void ode(double *dx, double t, const double *x, const double *u, const double *p);
	  bool ode_diff(DiffStructure &s, double t, const double *x, const double *u, const double *p);
	  bool ode_diff_p(DiffStructure &s, double t, const double *x, const double *u, const double *p, int index);

	  void x_init(double *x, int i, int dis);
	  void u_init(double *u, int i, int dis);


	  void x_boundary(double *x_low, double *x_upp);
	  void u_boundary(double *u_low, double *u_upp);

	  virtual void SC_ode(double *dx, double t, const double *x, const double *u, const double *p)=0;
	  virtual bool SC_ode_diff(DiffStructure &s, double t, const double *x, const double *u, const double *p);
	  virtual bool SC_ode_diff_p(DiffStructure &s, double t, const double *x, const double *u, const double *p, int index);

	  virtual void SC_x_boundary(double *x_low, double *x_upp);
	  virtual void SC_u_boundary(double *u_low, double *u_upp);

	  virtual void SC_x_init(double *x, int i, int dis);
	  virtual void SC_u_init(double *u, int i, int dis);


	  virtual void u_scale(double *u);
	  virtual void u_unscale(double *u);
	  virtual void x_scale(double *x);
	  virtual void x_unscale(double *x);
};



//void win_terminal(int fw, int fh, int bw, int bh);
//void win_resize(int fw, int fh, int bw, int bh);
void DllExport worhpoutput(int i, const char *message);
//void MyOutputFunction(ext_io_func f);

#endif
