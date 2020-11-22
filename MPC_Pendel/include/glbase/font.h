//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
#ifndef font_h
#define font_h
#include "../base/defines.h"
#include <GL/gl.h>
#include <string>
#include "xmlio.h"

/** @ingroup glbase
 *  @brief Interface for OpenGL Fonts.
 * 
 */
class FFont {
public:
	/** Konstruktor. */
	FFont() {}
	/** Destruktor. */
	virtual ~FFont() {}
	
	/** Ausgabe der Zeichenkette s an (x/y/z). */
	virtual void printString(const char *s, float x, float y, float z) const = 0;
	/** Ausgabe der Zeichenkette s an (x/y/z). */
	virtual void printString(const char *s, float x, float y, float z, int width) const = 0;
	
	/** Bestimmung der Breite der Zeichenkette. */
	virtual int StringLength(const char *s) const;
	
	
	static const unsigned char psi, phi, lambda,alpha,beta,gamma,delta,epsilon,Omega,omega;
	static const unsigned char UP, DOWN, STEP, BACK, DOT, UPUP, DOWNDOWN,CHECK,UNCHECK,GORIGHT;

	struct letter {
		letter(const std::string &text);
		letter(XMLNode *node);
		unsigned char character;
		int width;
		int height;
		GLubyte raster[10];
	};
	
	const letter* Key(unsigned char s) const;
	
protected:
	GLuint fontOffset;
	std::vector<letter> letters;
};

#endif
