#ifndef sdlcursor_h
#define sdlcursor_h
#include <SDL2/SDL.h>

#include "../base/point.h"

class SDLCursor {
public:
	SDLCursor() : sdl_cursor(0) {}
	void Init(const char *image[], const Point<int> &hot);
	static void CleanCursors();
	void Set();
 void Clean();
	SDL_Cursor *sdl_cursor;
	
	static void CreateCursors();
};

extern SDLCursor cursor[24];

#endif
