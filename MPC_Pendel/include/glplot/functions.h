#ifndef functions_h
#define functions_h

/**
 *  Use this typedef to implement your own data drawing routine, see also
 *  createuserwindow_().
 *  @param handle Window handle.
 *  @param frame Current frame to be drawn.
 *  @param len Number of available data sets.
 *  @param ndgl Dimension of x data set.
 *  @param nsteuer Dimension of u data set.
 *  @param t Time vector (of size len).
 *  @param x ODE vector (of size len*ndgl).
 *  @param u Control vector (of size len*nsteuer).
 *  @param bord Set this double[4]-Vector to provide min/max values for axes.
 */
typedef void (*Funktionenzeiger) (int &handle, int &frame,
                                  int &len, int &ndgl, int &nsteuer, double *t, double *x,
                                  double *u, double *bord);
typedef void (*FunktionenzeigerI) (int &handle, int &frame,
                                   int &len, int &ndgl, int &nsteuer, double *t, double *x,
                                   double *u, double *bord, int &index);
typedef void (*FunktionenzeigerU) (int &handle, int &frame,
                                   int &len, int &ndgl, int &nsteuer, int &nunbe, double *t, double *x,
                                   double *u, double *unknown, double *bord);
/**
 *  Use this typedef to draw more complex data sets by combining values from
 *  t, x and u, see also createdatawindow_().
 *  @param len Number of available data sets.
 *  @param ndgl Dimension of x data set.
 *  @param nsteuer Dimension of u data set.
 *  @param t Time vector (of size len).
 *  @param x ODE vector (of size len*ndgl).
 *  @param u Control vector (of size len*nsteuer).
 *  @param i Data at position i (out of len) should be returned.
 *  @param index Specify type of value.
 *  @return Calculated data of type index at position i.
 */
typedef double (*Funktionenzeiger2) (int &len, int &ndgl, int &nsteuer,
                                     double *t,double *x, double *u, int &i, int &index);
/** @} */
typedef void (*FunktionenzeigerC) (int &button, double &xpos, double &ypos,
                                   int &len, int &ndgl, int &nsteuer, double *t, double *x, double *u
                                  );
typedef double (*timefuncf) (int &, double *, double *, double *, int &, int &, int &);



#endif

