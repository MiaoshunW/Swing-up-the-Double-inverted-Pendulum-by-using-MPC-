//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef smoothmovement_h
#define smoothmovement_h


/** @ingroup glbase
 *  @brief Timed acceleration based movement.
 * 
 */
class SmoothMovement {

public:
	SmoothMovement();
	void Init(float minv, float maxv, bool r, int o100 = false);
	bool Timer(double dt);
	void Set(float v);
	void Stop() {velocity=0;value=0;}
	void Accelerate(int step, int maxspeed);
void JoyInput(int j);

	void Goto(float v, double time);

	
	float value;
	float value0;
	
	int only100;
	int velocity;

	float maxval,minval;
private:
	bool rot;

	float gotoval;
	double gototime;
};


#endif

