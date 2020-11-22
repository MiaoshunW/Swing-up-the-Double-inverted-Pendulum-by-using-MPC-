#ifndef TWconsole_h
#define TWconsole_h

#include "../base/defines.h"
class XMLNode;
#include "textout.h"


/** Manipulation der Win-Console */
class TWconsole {

public:
	TWconsole() : active(0), fw(6), fh(8), bw(132), bh(2000) {}
	
	void ParseXML(XMLNode *xmlmain);
	
	void Init();
	void Resize(int bh_);

	int Active() {return active;}
	
private:
	int active, fw, fh, bw, bh;
	

};

#endif
