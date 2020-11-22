#ifndef sdlframe_h
#define sdlframe_h
#include <GL/gl.h>
#include "../base/defines.h"
 
#include <SDL2/SDL.h>
#include <string>
#include "xmlio.h"

#include "sdlscreen.h"
class ImageWriter;

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

/** @ingroup gl
 *  @brief Create physical window under Linux and Windows with OpenGL context.
 */
class DllExport SDLFrame {
public:
	SDLFrame(SDLScreen* screen_);
	virtual ~SDLFrame();

	int Loop(int wait, int terminate);
	int Wait();
	void Timer();
	//void Info();

	void MyQuit(int status);
	
	virtual bool KeyboardFunc(SDL_Keysym &keysym);
	virtual bool SpecialKeys(const Uint8 *keys);
	virtual bool MouseButton(SDL_MouseButtonEvent &m);
	virtual bool MouseMotion(SDL_MouseMotionEvent &m);
	virtual bool JoystickButtonEvent(SDL_JoyButtonEvent &j);
	virtual bool JoystickAxisEvent(SDL_JoyAxisEvent &j);
	virtual bool JoystickHatEvent(SDL_JoyHatEvent &j);
	virtual void TimerFunc();


	virtual void Message(int id, const void *param=0);
	void Reshape(int w, int h);
	virtual void RenderScene();
	//virtual void RenderNamedScene()=0;

	
	void accPerspective(GLdouble fovy, GLdouble aspect,
			    GLdouble nnear, GLdouble ffar, GLdouble pixdx,
			    GLdouble pixdy, GLdouble eyedx, GLdouble eyedy, GLdouble focus);
	void accFrustum(GLdouble left, GLdouble right, GLdouble bottom,
			GLdouble top, GLdouble nnear, GLdouble ffar, GLdouble pixdx,
			GLdouble pixdy, GLdouble eyedx, GLdouble eyedy, GLdouble focus);
	
	
	SDLScreen *screen;
	
	int waittime;
	int returnvalue;
	
protected:

	virtual int Redraw();

	ImageWriter *imgwriter;
	int imgwriterflag;
	
private:

	bool render;
	int busy;
};

#endif
