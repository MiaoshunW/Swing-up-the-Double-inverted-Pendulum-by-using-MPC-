#ifndef sdlscreen_h
#define sdlscreen_h
#include "../base/defines.h"
#include <SDL2/SDL.h>
#include "xmlio.h"
#include "../base/point.h"

class SDLFrame;
class SDLThread;


struct TWwindow {
	
	TWwindow();
	
	void ParseXML(XMLNode *xml);
	
	/** Drawable OpenGL Area */
	Point<int> size;
	
	/** Physical screen resolution */
	Point<int> resolution;
	
	/** Displacement of upper left corner */
	Point<int> reference;
	
	bool fullscreen;
	
	int multisamplebuffers;
	int multisamplesamples;
	
	Uint32 video;
	
	/** 0: normal, 1: quiet */
	int windowmode;
};



/** @ingroup gl
 *  @brief Create physical screen under Linux and Windows.
 */
class SDLScreen {
public:
	SDLScreen(TWwindow *twscreen);
	virtual ~SDLScreen();

	void ToggleFullScreen(SDLFrame *parent);
	void SetIcon();
	
	int Width() const;
	int Height() const;

	SDLThread *thethread;
	SDL_Surface *image;
	
	SDL_Window *window;
	SDL_GLContext glcontext;

	TWwindow twwindow;
};

#endif
