#ifndef viewer_h
#define viewer_h

#include "../base/defines.h"

#include <vector>
#include <map>

//#include "../base/language.h"

#include "../glbase/smoothmovement.h"
#include "../glbase/light.h"
#include "../glbase/font.h"
#include "../glbase/joystick.h"
//#include "../glbase/ttfont.h"

#include "../gui/sdlframe.h"
#include "../gui/sdlcursor.h"
#include "../gui/sdlthread.h"

#include "../toolbase/toolmenu.h"

#include "../glplot/plot.h"
#include "../glplot/xopt_data.h"

#include "TransWORHP.h"

class glObject;
class ToolBox;
class TWfolder;


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

#ifdef NOGRAPHICS



#else

class DllExport Viewer : public SDLFrame {

public:
	Viewer(TWparameter *twparam);
	~Viewer();

	void RenderScene();

	void displayObjects();
	void display3dObjects();

	void displayText();
	void displayText00();

	void TimerFunc();
	bool SpecialKeys(const Uint8 *keys);
	bool KeyboardFunc(SDL_Keysym &keysym);

	/*bool JoystickTimer();
	bool JoystickAxisEvent(SDL_JoyAxisEvent &j);
	bool JoystickButtonEvent(SDL_JoyButtonEvent &j);
	bool JoystickHatEvent(SDL_JoyHatEvent &j);
	*/
	bool MouseButton(SDL_MouseButtonEvent &m);
	bool MouseMotion(SDL_MouseMotionEvent &m);

	void Message(int id, const void *param=0);

	void WriteZen();
		
	FFont *font;

	double fps;
	int framecount;

	std::string path;


	double xlasttime;

	std::vector<glObject*> objects;


	ToolMenu menu;


	void InitTools();
	void CreateMenu();

	std::vector<BasePlot *> plots;
	int animate;
	int SceneTime;
	double PlayerTime;
	double *x_now;
private:
	struct Background {
		Background();
		void Init(XMLNode *xml);
		void InitTexture();
		void displayMenu(int width, int height);
		void displayLogo(int width, int height);
		void display(int width, int height);
		int flagLogo;
		int flagGrid;

		bool waiting;
		int mouseOverButton;
		int border,border2;
		Texture logotex[3];
		color4 color1;
		color4 color2;
		color4 colorGrid;
	}
	bg;

	struct DragInfo {

		DragInfo() : plot(0) {}

		void Set(BasePlot *p,int m, const Point<int> &pt) {
			plot=p;
			geom = p->geom.cur;
			point = pt;
			mode = m;
		}
		BasePlot *plot;
		Point<int> point;
		int mode;
		BasePlot::Geometry geom;
	}
	drag;


public:
	/**
	* Update graphics
	*/
	void Update();
	void Update(TransWorhp *ph, int i, DataStorage *ds);
	/**
	* Wait for user
	*/
	//void Wait();
	/**
	* Close windows
	*/
	void CloseAll();

	void Init(TWfolder *p);
	
	void AddStateView(const int n, const std::string &name);
	void AddControlView(const int n, const std::string &name);
	void AddIntegralView(const int n, const std::string &name);
	void AddLambdaView(const int n, const std::string &name);
	void AddMuView(const int n, const std::string &name);
	
	void selectWindows();
	
	void Matrix(const char *s, WorhpMatrix *m);

	TWfolder *twfolder;

	std::vector<DataStorage*> dsvector;
	DataStorage dstop;

	
	double *temptime[10];
	
	void AutoScale();

	void AddPlot(BasePlot *p);

	size_t Size() const;
	
	BasePlot* Last() const;
	BasePlot* Get(int i) const;

	void SetTempTime();
	void TilePlots(double TT=500);
	BasePlot *mouseOver;
	BasePlot *modifyPlot;
	int StartTime;
	int CurrentTime;
	int change;
	int Running;

	void MaxPlot(BasePlot *p);
	void MouseFunc(int button, int state, const Point<int> &p);
	void PassiveMotionFunc(const Point<int> &p);
	void MotionFunc(const Point<int> &p);


	virtual int Redraw();

	void PhasePlot(const char *s, Funktionenzeiger2 func, int d1, int d2);
	void Data(const char *s, Funktionenzeiger2 func, int index);
	void ThreeD(const char*s, XMLNode *xml, plot3d f);

	void AddCompareCurve(int &cmpstep, double* cmp, double* cmptime, int n);
	void AddDynCompareCurve(int i, int *cmpstep, double* cmp, double* cmptime, int *n);
	void AddCompareCurve(int i, double* cmptime, double* cmp, int &cmpstep, int n);




	void SetHeader(char *s, int mode);
	std::string header;
	int headermode;

	void SetFloatTime(double t, double t0);
	void SetTimeIsTime();

	void GetState(double *x, double t);

	/* Matthias Rick */
	
	TWparameter *twparam;
	
	void disFehler(const std::vector<std::vector<double> > &zeit, const std::vector<std::vector<double> > &fehler, const TWdiscretization *twdiscretization);
	void restrictionPlot(const double *zeit, const double *lambda, int n_dis, int n_ctrl, int n_ode, const TWdiscretization *twdiscretization);
	void adjungiertenPlot(const double *zeit, const double *mu, int n_dis, int n_ctrl, int n_ode, int n_con, const TWdiscretization *twdiscretization);
};

#endif
#endif
