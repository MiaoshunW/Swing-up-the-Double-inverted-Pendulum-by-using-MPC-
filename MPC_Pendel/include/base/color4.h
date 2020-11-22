//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef color4_h
#define color4_h

#include "defines.h"
#include <string>
#include <iostream>

/** @ingroup base
    @brief Red-Green-Blue-Alpha-Color.
    
    color4 values can be read from a string, be transformed (dim, alpha) and be sent to OpenGL.
  */
class color4 {
public:
	/**
	 * Empty constructor. Set color to white
	 */
	color4() : name("White") {
		c[0] = 1.;
		c[1] = 1.;
		c[2] = 1.;
		c[3] = 1.;
	}
	/**
	 * Constructor. Set color.
	 * @param r red
	 * @param g green
	 * @param b blue
	 * @param a alpha
	 */
	color4(float r, float g, float b, float a=1.) {
		c[0] = r;
		c[1] = g;
		c[2] = b;
		c[3] = a;
	}
	/**
	 * Constructor. Extract color value from string.
	 * @param c String, "colorname alpha" or "r g b a"
	 */
	color4(const std::string &c);

	/**
	 * Set color to given hue value.
	 * @param hue Interpolate color between these values:
	      - 0.0: red
	      - 0.2: yellow
	      - 0.4: green
	      - 0.6: cyan
	      - 0.8: blue
	      - 1.0: magenta
	 */
	void SetHue(float hue);
	/**
	 * Set color to grey.
	 * @param val Intensity of grey.
	 */
	void SetGrey(float val);

	/**
	 * Mix current color with black
	 * @param a Amount of black added to color: 
	       - 0: 100% current color
	       - 1: 100% black
	 * @return Darkened color
	 */
	color4 Dim(float a) const;
	/**
	 * Mix current color with white
	 * @param a Amount of black added to color: 
	       - 0: 100% current color
	       - 1: 100% white
	 * @return Brightened color
	 */
	color4 Bright(float a) const;
	/**
	 * Make current color transparent
	 * @param a Transparency value
	       - 0: 100% transparent = invisible
	       - 1: opaque
	 * @return transparent color
	 */
	color4 Alpha(float a) const;


	/**
	 * Write color value to OpenGL by calling glColor4()
	 */
	void operator()() const;

	/**
	* Write value to text string.
	* @param v color4
	* @return 
	*/
	friend std::string ToString(const color4 &v);

	/**
	 * Get color data.
	 * @return 
	 */
	const float *GetData() const {
		return &c[0];
	}

	float alpha() const {
		return c[3];
	}
	float red() const {
		return c[0];
	}
	float green() const {
		return c[1];
	}
	float blue() const {
		return c[2];
	}

	bool operator==(const color4 &other) const {
		if (c[0]!=other.c[0])
			return false;
		if (c[1]!=other.c[1])
			return false;
		if (c[2]!=other.c[2])
			return false;
		if (c[3]!=other.c[3])
			return false;
return true;
	}

friend std::ostream& operator<<(std::ostream &os, const color4 &c);
	
private:
	float c[4];
	std::string name;
};


#endif
