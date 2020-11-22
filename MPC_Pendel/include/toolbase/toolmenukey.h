//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef toolmenukey_h
#define toolmenukey_h

#include <string>
#include "SDL2/SDL.h"


/** @ingroup toolbase
 *  @brief MenuKey
 *
 */
class ToolMenuKey {
public:
	ToolMenuKey() {}
	ToolMenuKey(const std::string &s, int i);
	
	bool Check(SDL_Keycode key, Uint16 mod);

private:
	SDL_Keycode k;
	//SDL_Keymod m;
	int id;
	int ctrl,alt,shift;
};

#endif


