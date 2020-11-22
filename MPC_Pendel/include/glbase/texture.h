//
// Author: Matthias Knauer <knauer@math.uni-bremen.de>, (C) 2007
//
// Copyright: See COPYING file that comes with this distribution
//
/***************************************************************************
 *   Copyright (C) 2004 by Matthias Knauer                                 *
 *   knauer@math.uni-bremen.de                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef texture_h
#define texture_h
#include "../base/defines.h"
#include <GL/gl.h>
#include <string>

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

/** @ingroup glbase
 *  @brief Loading Textures for OpenGL, from PNG files.
 * 
 */
class DllExport Texture {
public:
	Texture();

	virtual ~Texture();

	void LoadTextureRAW(const char * filename, int wrap, int w, int h=1, int dp=3);
	//void SaveTexture(std::ostream &os, const std::string &name, int dd=1);
	void LoadTexture( GLubyte *data, int wrap, int w, int h=1, int dp=3 );
	void LoadTexturePNG(const std::string &filename, int wrap, bool auto_alpha=false );

	int GetData(int x, int y, int d=0) const {
		return data[(x+width*y)*depth+d];
	}
	void Bind() const;
	int GetWidth() const {
		return width;
	}
	int GetHeight() const {
		return height;
	}

	/*void Reset() {
		delete []data;
		data=0;
	}*/

void GrabScreen(int dx, int dy, int w, int h);

	GLuint texture;
private:
	GLenum format;
	GLubyte *data;int dataw,datah;
	int width;
	int height;
	int depth;
	int dim;
	
};

#endif
