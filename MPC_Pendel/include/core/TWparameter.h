#ifndef TWparameter_h
#define TWparameter_h

#include "xmlio.h"
#include "TWconsole.h"

#ifndef NOGRAPHICS
#include "../gui/sdlscreen.h"
#else 

struct TWwindow {
	
	TWwindow() {}
	
};

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

	
/** Unterscheiden der Diskretisierung für TransWORHP */
class DllExport TWdiscretization {
public:

	// Standard = Trapez
	TWdiscretization(DiscretizationType_e t=TW_Trapez, int s=1, int i=0) : type(t), stufen(s), innenpunkt(i) {}
	TWdiscretization(TWdiscretization& other) : type(other.type), stufen(other.stufen), innenpunkt(other.innenpunkt) {}

	/** Anzahl der Punkte der Intervalle 0 bis n_dis */
	int stuetzstellen(int n_dis) const {
		return (1+innenpunkt)*(n_dis-1)+1;
	}
	
	/** Anzahl der Punkte pro Intervall */
	int punkte() const {
		return 1+innenpunkt;
	}

	DiscretizationType_e type;
	
	int stufen;
	int innenpunkt;
};




class DllExport TWparameter {
public:
	TWparameter(const char *filename);
	~TWparameter();
	
	std::map<std::string,std::string>  Arguments(int argv, char* argc[]);
	
	void ParseXML(XMLNode *xmlmain);
	static XMLNode *ReadParams(const char *filename);

	/** Info über aktuelles Diskretisierungsschema */
	TWdiscretization twdiscretization;


	int butchertableau;
	double stepsize;
	double abserr, relerr;
	int linInter; /**< lineare Interpolation der Steuerung: 1=an, sonst=konst Steuer */
	int parallel; /**< paralleles Intergrieren mit openMP: 1=an, sonst=aus */

	/** Berechnungsart der Hessematrix */
	int hessianvalues;
	/** Struktur der Hessematrix */
	int hessianstructure;


	std::string paramfile_worhp;
	int USERDF, USERDG, USERHM;

	int SHOWDF, SHOWDG, SHOWHM;
	double eps;

	TWconsole twconsole;
	TWwindow twwindow;
	
	int NDIS;
	int PLOT;
	
	//Matthias Rick
	int meshref_mod; // Modus
	int meshref_err_mod; // Fehlerberuchnung Modus
	int meshref_M1; // Betts: M_1 Anz. neuer Punkte pro Intervall
	int meshref_R; //Betts: Ordnungsreduktion
	int meshref_M; //Betts: Anz. neuer Punkte Insgesamt
	int meshref_K; // max. Anz. an Gitteranpassungsschritten
	double meshref_kappa; // Betts: kappa
	double meshref_tol; // Fehlerschranke
	
	int meshref_VER; // Zwischenschritte der Verfeinerung
	
	int meshref_PLOT_SW; //Plot Schrittweite
	int meshref_PLOT_ERR; //Plot Fehler
	int meshref_PLOT_MP; //Plot Position der Gitterpunkte
	int meshref_PLOT_LAMBDA; //Plot Beschraenkungen
	int meshref_PLOT_MU; //Plot Adjungierte
	
	XMLNode *xml;
	
	std::vector<int> multinode;
};


#endif
