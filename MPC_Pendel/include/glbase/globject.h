#ifndef globject_h
#define globject_h
#include "../base/defines.h"
#include <string>
#include <GL/gl.h>
#include "xmlio.h"

#include "../base/vektor.h"

#include "model.h"


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

class DllExport glObject {
public:
	glObject();
	virtual ~glObject();

	void Init(XMLNode *n);
	void InitObject(const std::string &path);
	void InitSplitObject(const std::string &path);
	void Draw(Vektor<float>&pos, float phi, float psi, int i=1);

	
	std::string id;
int countObj() {return splitObjects;}
private:
	GLuint list;
	double scale;
	std::string filename;
Model *m2;
	double normalangle;
int splitObjects;
	//Model m;

};

#endif
