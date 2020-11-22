#ifndef viewport_h
#define viewport_h

#include "../base/point.h"
#include "texture.h"
#include "../base/color4.h"

struct rectangle {

	rectangle() : x(0),y(0),width(0),height(0) {}
	rectangle(int x_,int y_, int w_, int h_) : x(x_),y(y_),width(w_),height(h_) {}

	void drawTexRect(const Point<int> &r, const Texture &tex, int sc=1) {

		float ww = (float)tex.GetWidth()/sc;
		float hh = (float)tex.GetHeight()/sc;

		glTexCoord2f(r.x/ww           ,r.y/hh);
		glVertex3f((float)x           ,(float)(y+height),0.f);

		glTexCoord2f(r.x/ww           ,(r.y+height)/hh);
		glVertex3f((float)x           ,(float)y,0.f);

		glTexCoord2f((r.x+width)/ww   ,(r.y+height)/hh);
		glVertex3f((float)(x+width)   ,(float)y,0.f);

		glTexCoord2f((r.x+width)/ww   ,r.y/hh);
		glVertex3f((float)(x+width)   ,(float)(y+height),0.f);
	}

	void drawRect() {

		glVertex3i(x              ,(y+height),0);
		glVertex3i(x              ,y,0);
		glVertex3i((x+width)    ,y,0);
		glVertex3i((x+width)    ,(y+height),0);
	}
	void drawLineRect() {

		glVertex3i(x              ,(y+height),0);
		glVertex3i(x              ,y,0);
		glVertex3i((x+width)    ,y,0);
		glVertex3i((x+width)    ,(y+height),0);
		glVertex3i(x              ,(y+height),0);
	}
	void drawColorRect(const color4 &c1, const color4 &c2) {

		c1();
		glVertex3f((float)x              ,(float)(y+height),0.f);
		c2();
		glVertex3f((float)x              ,(float)y,0.f);
		glVertex3f((float)(x+width)    ,(float)y,0.f);
		c1();
		glVertex3f((float)(x+width)    ,(float)(y+height),0.f);
	}


	int x;
	int y;
	int width;
	int height;

};

struct Viewport {

	Viewport() : dx(0),dy(0),neigung(0),roll(0) {}
	double Ratio() {
		return (double)r.width/(double)r.height;
	}
	void Set();
	void SetLeft();
	void SetRight(int w);

	
	void Position();
	//void Frame(int h);

	rectangle r;

	double tau,yang,xang,dx,dy;
	double neigung,roll;

};

#endif

