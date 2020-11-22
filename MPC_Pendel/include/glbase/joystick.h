#ifndef joystick_h
#define joystick_h
#include "../base/defines.h"
#include "SDL2/SDL.h"
#include <string>

class Joystick {

public:
	Joystick();
	~Joystick();

	bool Valid() {return (joy!=0);}
	int GetAxis(int axis);
	bool GetButton(int button);
	int GetHat(int hat);
	//void ButtonEvent(SDL_JoyButtonEvent &e);

	int GetIndex() {return index;}
private:
	SDL_Joystick *joy;
	int index;
	std::string name;
	int numhats;

};

#endif

