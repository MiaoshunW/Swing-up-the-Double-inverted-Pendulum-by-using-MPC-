//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef toolmenu_h
#define toolmenu_h

#include <vector>
#include <string>
#include "../base/point.h"
#include "SDL2/SDL.h"
#include "toolmenukey.h"
#include "toolstatus.h"
#include "../glbase/smoothmovement.h"

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

class ToolMenuEntry;

/** @ingroup glbase
 *  @brief Menu
 *
 */
class DllExport ToolMenu {

public:
	ToolMenu();
	virtual ~ToolMenu();
	void Draw(int w) const;

	ToolMenuEntry *AddMenu(const std::string &s, int id=0);
	//MenuEntry *AddMenu(MenuEntry *th, const std::string &s, int id);
	ToolMenuEntry *AddMenu(ToolMenuEntry *th, const std::string &s, int id, const std::string &hot="", int *selected=0);
	ToolMenuEntry *AddMenu(ToolMenuEntry *th, const std::string &s, int id, float *val, float minval, float maxval, const std::string &hot="");
	ToolMenuEntry *AddMenu(ToolMenuEntry *th, const std::string &s, int id, SmoothMovement &sm, const std::string &hot="");

	ToolMenuEntry *AddSeparator(ToolMenuEntry *th);

	bool MouseButton(SDL_MouseButtonEvent &m);
	bool MouseMotion(SDL_MouseMotionEvent &m);
	void Timer(int t);
	bool KeyboardFunc(SDL_Keysym &keysym);
	bool SpecialKeys(const Uint8 *keys);

	ToolStatus status;
	
std::string infotext;

private:
	std::vector<ToolMenuEntry*> me;
	ToolMenuEntry *openentry;
	int opened;

	int visible;
	int yy;

	std::vector<ToolMenuKey> mk;

	int menuposct;
	
	
};

#endif
