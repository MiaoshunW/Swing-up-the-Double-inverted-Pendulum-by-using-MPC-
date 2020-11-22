#ifndef XOPT_DATA_H
#define XOPT_DATA_H

#include "../base/defines.h"
#include "functions.h"
#include "GL/gl.h"


struct Selector {
	
	Selector(char c_, int n_) : c(c_),n(n_) {}
	
	char c; // 't', 'x', 'u', 'l', 'g'
	int n;  // 0..
};							   
	


/** @defgroup data Data Access and Storage
 *  ???
 * @{
 */
class DataStorage;

/** Treat FORTRAN vector as 2-dimensional vector */
class DataVector {
	friend class DataStorage;
public:
	/** Constructor */
	DataVector() : ndis(0), size(0), offset(0), D(0) { }
	/** Destructor */
	~DataVector() {}

	void SetVector(int n, int s, int off, double *value) {
		ndis=n;
		size=s;
		offset=off;
		D=value;
	}
	double GetData(int dgl, int index) const {
		return D[offset*index+dgl];
	}
	void SetData(int dgl, int index, double v);
	void Freeze();
	void UnFreeze() {
		delete D;
		D=0;
		ndis=0;
		size=0;
	}
	void Copy(DataVector &other);
	int GetSize() const {
		return size;
	}

//private:

	/** Anzahl der diskreten Daten */
	int ndis;
	
	/** Werte pro diskretem Punkt */
	int size;
	
	/** SpeicherOffset zu nächsten Datenpunkt */
	int offset;
	
	double *D;
};

/** Contains vectors for time, x, u, g */
class DataStorage {
public:
	DataStorage() : tfgesamt(1), length(0) {}
	~DataStorage() {}
	
	void SetData(int len, 
	          double *t, 
			  double *x, int xoffset, int xdata,
			  double *u, int uoffset, int udata,
			  double *g, int goffset, int gdata,
			  double *l, int loffset, int ldata
				);
	
	//void SetData2(int nunbe, double *unknown);

	void Freeze() {
		T.Freeze();
		X.Freeze();
		U.Freeze();
		G.Freeze();
		L.Freeze();
	}
	void UnFreeze() {
		T.UnFreeze();
		X.UnFreeze();
		U.UnFreeze();
		G.UnFreeze();
		L.UnFreeze();
	}
	void Copy(DataStorage &ds) {
		T.Copy(ds.T);
		X.Copy(ds.X);
		U.Copy(ds.U);
		G.Copy(ds.G);
		L.Copy(ds.L);
	}

	double GetData(const Selector &data,int index) const;
	void SetData(const Selector &data,int index, double val);
	double GetTF() const {
		return tfgesamt;
	}
	int GetLength() const {
		return length;
	}

	void Call(Funktionenzeiger Func, int index, int f, double *bord) {
		(*Func)(index,f,length,X.size,U.size,T.D,X.D,U.D,bord);
	}
	void Call(FunktionenzeigerI Func, int index, int f, double *bord,int iindex) {
		(*Func)(index,f,length,X.size,U.size,T.D,X.D,U.D,bord,iindex);
	}
	double Call(Funktionenzeiger2 Func, int i, int param) {
		return (*Func)(length,X.size,U.size,T.D,X.D,U.D,i,param);
	}
	void Call(FunktionenzeigerC Func, int button, double xpos, double ypos) {
		(*Func)(button,xpos,ypos,length,X.size,U.size,T.D,X.D,U.D);
	}
	void Call(FunktionenzeigerU Func, int index, int f, double *bord) {
		(*Func)(index,f,length,X.size,U.size,UNKNOWN.size,T.D,X.D,U.D,UNKNOWN.D,bord);
	}

	enum TimeMode_e {index_as_time, dgl_as_time, fixed_float_as_time, dgl2_as_time, time_is_time,time_as_func};

	static TimeMode_e timemode;
	static int timedgl;
	static double floattime;
	static double floatstart;
	
	static int nsplitdis;
	static int nsplitlen;
	static int nsplitdis2;
	static int *nsplitindex;
	static double MaxTF;

//private:
	DataVector U,X,T,G,L,UNKNOWN;
	double tfgesamt;
	int length;
 int nunbe;
 double start;


};


double CallTimeFunc(int &i,double *X, double *UNBE, double *T,
	                                int &len, int &ndgl, int &nunbe);

//extern DataStorage ds;
//extern DataStorage dstop;

/** @} */

#endif // XOPT_DATA_H

