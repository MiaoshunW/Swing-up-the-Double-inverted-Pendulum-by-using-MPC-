//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef light_h
#define light_h

#ifdef WIN32
#include "windows.h"
#endif

#include <GL/gl.h>

#include "../base/color4.h"

class XMLNode;

/** @ingroup glbase
 *  @brief OpenGL Light.
 * 
 */
class Light {

public:
	Light();

	void Init(GLenum l, XMLNode *xml);

	void Set();
	void Position();
	void Direction();

	void Enable();
	void Disable();

	void SetPos(float x, float y, float z, float a=0) {
		position[0] = x;
		position[1] = y;
		position[2] = z;
		position[3] = a;
	}
	void ResetPos() {
		position[0] = pos0[0];
		position[1] = pos0[1];
		position[2] = pos0[2];
		position[3] = pos0[3];
	}

	void SetDir(float x, float y, float z, float a=0) {
		direction[0] = x;
		direction[1] = y;
		direction[2] = z;
		direction[3] = a;
	}

//private:
	GLenum light;

	GLfloat position[4];
	GLfloat pos0[4];
	GLfloat direction[4];

	color4 ambientLight;
	color4 diffuseLight;
	color4 specularLight;

	GLfloat spotCutoff;
	GLfloat spotExponent;
};

#endif

