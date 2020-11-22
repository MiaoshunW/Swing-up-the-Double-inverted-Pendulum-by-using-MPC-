/*----------------------------------------------------------------
 *
 * MPC_Doppelpendel:Test MPC controller for the double inverse pendulum
 *
 *---------------------------------------------------------------- */

#ifdef WIN32
#include "windows.h"
#endif
//#define NOGRAPHICS -> Buggy!
#include <conio.h>
#include "TransWORHP.h"

using namespace std;

const double Pi =     3.1415926535898;
const double Pi2 =    6.2831853071796;
const double Pi2inv = 0.159154943091895;


double       start[] = { 0, 0, Pi, Pi, 0, 0, 0 };
const double  ziel[] = { 0, 0,  0, Pi, 0, 0, 0 };        //Pendulum 2 should hang down  <Reference>
double  start_real[] = { 0, 0, Pi, Pi, 0, 0, 0 };
double      weight[] = { 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.02, 1.0 };
// anglespeed 1，anglespeed 2，angle1，angle2，cartposition，cartspeed， input power，pendulum energy    ？
// are each normalized to limit, or Pi

const double up_bnd[] = {  1.2,  25,  25,  1e20,  1e20,  0.38,  1.2, 0.5 };				// Set limits for pendulum 2  <constraints>
const double lw_bnd[] = { -1.2, -25, -25, -1e20, -1e20, -0.38, -1.2, -0.5 };			// Set limits for pendulum 2  <constraints>
				// Target speed, pendulum speed 1, pendulum speed 2, pendulum position 1, pendulum position 2, carriage position, carriage speed, point energy
const double dis_times[] = { 0, 0.1, 0.21, 0.42, 0.63, 0.84, 1.05 };


// if the current computation time         当前的计算时间 is lower than pre_calc_time less than
// executes the estimated computation time   执行估计的计算时间, the next start estimation does not become
// started only at the time of the control sequence, but directly
const double pre_calc_time = 0.0; //当前的计算时间
 //Original assumption of the calculation period
const double ext_estim_time = 0.1;   // 执行估计的计算时间                     // may increase, especially for the first step  //set to zero  ???

double control[6];	    //控制矩阵
double states[6][7];	//状态矩阵		
int error;

double gl, J_eff;
double max_bnd[7];

inline double wrap2pi(double angle) {
	return (angle - Pi2 * floor(angle*Pi2inv + 0.5));                               //  to simple the angle like 3pi= 1pi
}

inline double interp1(const double x[2], double y[2], double xi) {                  //  transfer the continuous mode to discrete mode
	return (((xi - x[0])*y[1] + (x[1] - xi)*y[0]) / (x[1] - x[0]));

}
// model constants
const double  a[2] = { 81.48 , 5135.0 };                        //  ???
const double  g = 9.81;		//gravitation
const double  T1 = 0.0395;	// time constant [s]
const double  a1 = 0.085;	// Half pendulum neck 1 [m]
const double  a2 = 0.157;	// Half pendulum neck 2 [m]
const double  l1 = 0.17;	// pendulum neck  1 [m]
const double  l2 = 0.314;	// pendulum neck  2 [m]
const double  m = 0.5;		// Cart mass [kg]
const double  m1 = 0.162;	// Mass pendulum 1 [kg]
const double  m2 = 0.203;	// Mass pendulum 2 [kg]
const double  J1 = ((m1 * l1 * l1) / 12);	    //Moment of inertia pendulum 1 [kg m^2]  	// 12--> 3 wegen Steiner Satz geschieht spaeter in Jeff
const double  J2 = ((m2 * l2 * l2) / 12);	    //Moment of inertia pendulum 2 [kg m^2]// Composite constants
const double  h1 = m + m1 + m2;
const double  h2 = m1*a1 + m2*l1;
const double  h3 = m2*a2;
const double  h4 = m1*a1*a1 + m2*l1*l1 + J1;
const double  h5 = m2*a2*l1;                      //h5=h3*l1
const double  h6 = m2*a2*a2 + J2;                 //h6=h3*a2+J2
const double  h7 = m1*a1*g + m2*l1*g;             // h7=h2*g
const double  h8 = m2*a2*g;                       //h8=h3*g
class MPC_Doppelpendel_optim : public TransWorhp {
public:

	MPC_Doppelpendel_optim(int dis) : TransWorhp("MPC_Doppelpendel_optim", dis, 7, 1, 0, 0, 0) {}

	void GetXTitle(int d, char *s) {
		if (d == 0) strcpy(s, "angle velocity of pendulum 1");
		if (d == 1) strcpy(s, "angle velocity of pendulum 2");
		if (d == 2) strcpy(s, "angle of pendulum 1");
		if (d == 3) strcpy(s, "angle of pendulum 2");
		if (d == 4) strcpy(s, "cart displacement");
		if (d == 5) strcpy(s, "cart velocity");
		if (d == 6) strcpy(s, "input energy");
	}
	void GetUTitle(int d, char *s) {
		if (d == 0) strcpy(s, "input velocity");                   //
	}


	//--------- Start optimization (optional) ---------------------//
	void x_init(double *x, int i, int dis) {
		for (j = 0; j < n_ode; j++) {								//n_ode:number of states
			//	x[j] = (start[j] * (dis - i) + ziel[j] * i) / dis;  	//x[j] = (start[j] * (dis - i) + ziel[j] * i)/ dis ;
			 x[j] = states[i][j];
			 //x[j] = start[j];
		}
	}
	void u_init(double *u, int i, int dis) {
		u[0] = 0.0;
		//u[0] = control[i];
	}


	//--------- Objective Function -------------------------------------------//

	double obj() {                                                                                  //???
		double tmp, alpha, ret;
		j = n_dis - 1;                              //格点数    number of grid points  
		tmp = (x(j, 5) - ziel[5]);
		ret= tmp*tmp * weight[5];				   	// cart speed         // Angel 1 speed，Angel 2 speed，Angel1 ，Angel2，cart position，cart speed

		//alpha = fabs(fmod(x(j, 2) - ziel[0] + Pi, Pi2) - Pi);
		tmp = fabs(fmod(x(j, 2) - ziel[2] + Pi, Pi2) - Pi);
		ret += tmp*tmp * weight[2];                 // pendulum angle 1     Change here without angle perhapes

		tmp = fabs(fmod(x(j, 3) - ziel[3] + Pi, Pi2) - Pi);
		ret += tmp*tmp * weight[3];                 // pendulum angle 2

		tmp = (x(j, 0) - ziel[0]);
		ret += tmp*tmp * weight[0];					// Pendulum angle speed 1

		tmp = (x(j, 1) - ziel[1]);
		ret += tmp*tmp * weight[1];					// Pendulum angle speed 2


		tmp = (x(j, 6) - ziel[6]);					// input power
		ret += tmp * weight[6];

		tmp = cos(x(j, 2)) - cos(ziel[2]);
		tmp *= l1*g*m2 + g*m1*a1;		                            // Here pendulum energy is calculated, here most likely to make changes! what about h7 / h8?
		tmp += cos(x(j, 3)) - cos(ziel[3])*a2*m2*g;
		ret += tmp*tmp* weight[7];					                // pendulum energy 
		//weight7  from  V = Vcart + Vpendulum1 + Vpendulum2 = T1 + T2 + T3 = 0 + m1*g*a1*cosθ1 + m2g(l1*cosθ1 + a2*cosθ2)
		//ret *= alpha / Pi;							 // standardization
		tmp = (x(j, 4) - ziel[4]);
		ret += tmp*tmp * weight[4];					 // carriage position

		return ret;
	}

	bool obj_structure(DiffStructure &s) {
		s(0, x_indexode(0));
		s(0, x_indexode(1));
		s(0, x_indexode(2));
		s(0, x_indexode(3));
		s(0, x_indexode(4));
		s(0, x_indexode(5));
		s(0, x_indexode(6));
		return true;
	}

	bool obj_diff(DiffStructure &s) {
		s(0, x_indexode(0)) = 2 * (x(j, 0) - ziel[0])* weight[0];
		s(0, x_indexode(1)) = 2 * (x(j, 1) - ziel[1])* weight[1];
		s(0, x_indexode(2)) = 2 * (fmod(x(j, 2) - ziel[2] + Pi, Pi2) - Pi)* weight[2];
		s(0, x_indexode(2)) += -2 * fabs(cos(x(j, 2)) - cos(ziel[2]))*(l1*g*m2 + g*m1*a1)*sin(x(j, 2))*weight[7];
		s(0, x_indexode(3)) = 2 * (fmod(x(j, 3) - ziel[3] + Pi, Pi2) - Pi)* weight[3];
		s(0, x_indexode(3)) += 2 * (cos(x(j, 3)) - cos(ziel[3])*a2*m2*g)*(-sin(x(j, 3)))*weight[7];
		s(0, x_indexode(4)) = 2 * (x(j, 4) - ziel[4])* weight[4];
		s(0, x_indexode(5)) = 2 * (x(j, 5) - ziel[5])* weight[5];
		s(0, x_indexode(6)) = weight[6];
		return true;
	}

	//--------- System model ---111---------------------------------------------//

	double det;
	double M11_inv[2][2];
	double M12[2];
	double C1[2];
	double G1[2];
	double N1;
	double N2;
	double def;
	double P;

	void ode(double *dx, double t, const double *x, const double *u, const double *p) {		// possibly shorten

		det = (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3]));

		M11_inv[0][0] = h6 / det;
		M11_inv[0][1] = (-1 * h5 * cos(x[2] - x[3])) / det;
		M11_inv[1][0] = (-1 * h5 * cos(x[2] - x[3])) / det;
		M11_inv[1][1] = h4 / det;

		M12[0] = h2 * cos(x[2]);
		M12[1] = h3 * cos(x[3]);

		C1[0] = h3*l1 * x[1] * x[1] * sin(x[2] - x[3]);                                                  //h5=h3*l1
		C1[1] = -1 * h3*l1 * x[0] * x[0] * sin(x[2] - x[3]);                                          // //h5=h3*l1

		G1[0] = -1 * h7 * sin(x[2]);                                                                    // h7=h2*g
		G1[1] = -1 * h8 * sin(x[3]);
		dx[5] = -1 * (x[5] - u[0]) / T1;
		N1 = (M12[0] * dx[5] + C1[0] + G1[0]);
		N2 = (M12[1] * dx[5] + C1[1] + G1[1]);
		dx[0] = -1 * M11_inv[0][0] * N1 - M11_inv[0][1] * N2;
		dx[1] = -1 * M11_inv[1][0] * N1 - M11_inv[1][1] * N2;
		dx[2] = x[0];
		dx[3] = x[1];
		dx[4] = x[5];
		dx[6] = u[0] * u[0];
	}
	//--------- Jacobian for Derivatives (Optional) -------------------------//
	bool ode_structure(DiffStructure &s) {
		//return false;
		s(0, x_indexode(0));
		s(0, x_indexode(1));
		s(0, x_indexode(2));
		s(0, x_indexode(3));
		s(0, x_indexode(5));
		s(0, u_indexode(0));

		s(1, x_indexode(0));
		s(1, x_indexode(1));
		s(1, x_indexode(2));
		s(1, x_indexode(3));
		s(1, x_indexode(5));
		s(1, u_indexode(0));

		s(2, x_indexode(0));

		s(3, x_indexode(1));

		s(4, x_indexode(5));

		s(5, x_indexode(5));
		s(5, u_indexode(0));

		s(6, u_indexode(0));

		return true;
	}
	bool ode_diff(DiffStructure& s, double t, const double* x, const double* u, const double* p) {
		//return false;
		def = (h4 * h6 - h5 * h5 * cos(x[2] - x[3])* cos(x[2] - x[3]));
		P = h5 * h5 * sin(2 * (x[2] - x[3]));

		//dx[0] = -1 * M11_inv[0][0] * N1 - M11_inv[0][1] * N2;
		s(0, x_indexode(0)) = -(2 * x[0] * h5 * h5 * cos(x[2] - x[3]) * sin(x[2] - x[3])) / def;
		s(0, x_indexode(1)) = -(2 * x[1] * h5 * h6 * sin(x[2] - x[3])) / def;
		s(0, x_indexode(2)) = h2*h6*(u[0] - x[5])*(sin(x[2])*def + P*cos(x[2])) / (T1*def*def) - h5*h6*x[1] * x[1] * (cos(x[2] - x[3])*def - P*sin(x[2] - x[3])) / (def*def) + h6*h7*(cos(x[2])*def - P*sin(x[2])) / (def*def) - h3*h5*(u[0] - x[5])*cos(x[3])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def*T1) - h5*h5*x[0] * x[0] * (2 * cos(2 * (x[2] - x[3]))*def - P*sin(2 * (x[2] - x[3]))) / (2 * def*def) + h5*h8*sin(x[3])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def);
	  //s(0, x_indexode(2))=  h2*h6*(u[0] - x[5])*(sin(x[2])*def + P*cos(x[2])) / (T1*def*def) - h5*h6*x[1] * x[1] * (cos(x[2] - x[3])*def - P*sin(x[2] - x[3])) / (def*def) + h6*h7*(cos(x[2])*def - P*sin(x[2])) / (def*def) - h3*h5*(u[0] - x[5])*cos(x[3])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def*T1) - h5*h5*x[0]*  x[0] * (2 * cos(2 * (x[2] - x[3]))*def - P*sin(2 * (x[2] - x[3]))) / (2 * def*def) + h5*h8*sin(x[3])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def);
		s(0, x_indexode(3)) = -h2*h6*cos(x[2])*(u[0] - x[5])*P / (T1*def*def) - h6*h5*x[1] * x[1] * (-cos(x[2] - x[3])*def + P*sin(x[2] - x[3])) / (def*def) + h6*h7*sin(x[2])*P / (def*def) + h5*h3*(u[0] - x[5])*(sin(x[2] - 2 * x[3])*def + P*cos(x[2] - x[3])*cos(x[3])) / (T1*def*def) - h5*h5*x[0] * x[0] * (-2 * cos(2 * (x[2] - x[3]))*def + P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h8*h5*(cos(x[2] - 2 * x[3])*def + P*cos(x[2] - x[3])*sin(x[3])) / (def*def);
	  //s(0, x_indexode(3)) = -h2*h6*cos(x[2])*(u[0] - x[5])*P / (T1*def*def) - h6*h5*x[1] * x[1] * (-cos(x[2] - x[3])*def + P*sin(x[2] - x[3])) / (def*def) + h6*h7*sin(x[2])*P / (def*def) + h5*h3*(u[0] - x[5])*(sin(x[2] - 2 * x[3])*def + P*cos(x[2] - x[3])*cos(x[3])) / (T1*def*def) - h5*h5*x[0] * x[0] * (-2 * cos(2 * (x[2] - x[3]))*def + P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h8*h5*(cos(x[2] - 2 * x[3])*def + P*cos(x[2] - x[3])*sin(x[3])) / (def*def);
		s(0, x_indexode(4)) = 0;
		s(0, x_indexode(5)) = (h2 * h6 * cos(x[2]) - h3 * h5 * cos(x[3]) * cos(x[2] - x[3])) / (T1 * (def));
		s(0, u_indexode(0)) = -(h2 * h6 * cos(x[2]) - h3 * h5 * cos(x[3]) * cos(x[2] - x[3])) / (T1 * (def));

		// dx[1]=dx[1] = -1 * M11_inv[1][0] * N1 - M11_inv[1][1] * N2;
		s(1, x_indexode(0)) = (2 * x[0] * h4 * h5 * sin(x[2] - x[3])) / (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3]));
		s(1, x_indexode(1)) = (2 * x[1] * h5 * h5 * cos(x[2] - x[3]) * sin(x[2] - x[3])) / (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3]));
		s(1, x_indexode(2)) = h5*h2*(u[0] - x[5])*((-cos(x[2] - x[3])*sin(x[2]) - sin(x[2] - x[3])*cos(x[2]))*def - P*cos(x[2])*cos(x[2] - x[3])) / (T1*def*def) + h5*h5*x[1] * x[1] * (2 * cos(2 * (x[2] - x[3]))*def - P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h5*h7*(cos(2 * x[2] - x[3])*def - P*cos(x[2] - x[3])*sin(x[2])) / (def*def) + h4*h3*(u[0] - x[5])*P*cos(x[3]) / (T1*def*def) + h5*h4*x[0] * x[0] * (cos(x[2] - x[3])*def - P*sin(x[2] - x[3])) / (def*def) - h4*h8*sin(x[3])*P / (def*def);
	  //s(1, x_indexode(2)) = h5*h2*(u[0] - x[5])*((-cos(x[2] - x[3])*sin(x[2]) - sin(x[2] - x[3])*cos(x[2]))*def - P*cos(x[2])*cos(x[2] - x[3])) / (T1*def*def) + h5*h5*x[1] * x[1] * (2 * cos(2 * (x[2] - x[3]))*def - P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h5*h7*(cos(2 * x[2] - x[3])*def - P*cos(x[2] - x[3])*sin(x[2])) / (def*def) + h4*h3*(u[0] - x[5])*P*cos(x[3]) / (T1*def*def) + h5*h4*x[0] * x[0] * (cos(x[2] - x[3])*def - P*sin(x[2] - x[3])) / (def*def) - h4*h8*sin(x[3])*P / (def*def);
		s(1, x_indexode(3)) = h2*h5*(u[0] - x[5])*cos(x[2])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def*T1) + h5*h5*x[1] * x[1] * (-2 * cos(2 * (x[2] - x[3]))*def + P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h5*h7*sin(x[2])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def) - h4*h3*(u[0] - x[5])*(-sin(x[3])*def + P*cos(x[3])) / (T1*def*def) + h4*h5*x[0] * x[0] * (-cos(x[2] - x[3])*def + P*sin(x[2] - x[3])) / (def*def) + h4*h8*(cos(x[3])*def + P*sin(x[3])) / (def*def);
	  //s(1, x_indexode(3)) = h2*h5*(u[0] - x[5])*cos(x[2])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def*T1) + h5*h5*x[1] * x[1] * (-2 * cos(2 * (x[2] - x[3]))*def + P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h5*h7*sin(x[2])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def) - h4*h3*(u[0] - x[5])*(-sin(x[3])*def + P*cos(x[3])) / (T1*def*def) + h4*h5*x[0] * x[0] * (-cos(x[2] - x[3])*def + P*sin(x[2] - x[3])) / (def*def) + h4*h8*(cos(x[3])*def + P*sin(x[3])) / (def*def);

		s(1, x_indexode(4)) = 0;
		s(1, x_indexode(5)) = ((h3 * h4 * cos(x[3])) - (h2 * h5 * cos(x[2]) * cos(x[2] - x[3]))) / (T1 * (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3])));
		s(1, u_indexode(0)) = -((h3 * h4 * cos(x[3])) - (h2 * h5 * cos(x[2]) * cos(x[2] - x[3]))) / (T1 * (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3])));

		// dx[2]= x[0];
		s(2, x_indexode(0)) = 1;
		s(2, x_indexode(1)) = 0;
		s(2, x_indexode(2)) = 0;
		s(2, x_indexode(3)) = 0;
		s(2, x_indexode(4)) = 0;
		s(2, x_indexode(5)) = 0;
		s(2, u_indexode(0)) = 0;

		// dx[3]= x[1];
		s(3, x_indexode(0)) = 0;
		s(3, x_indexode(1)) = 1;
		s(3, x_indexode(2)) = 0;
		s(3, x_indexode(3)) = 0;
		s(3, x_indexode(4)) = 0;
		s(3, x_indexode(5)) = 0;
		s(3, u_indexode(0)) = 0;

		// dx[4]=x[5];
		s(4, x_indexode(0)) = 0;
		s(4, x_indexode(1)) = 0;
		s(4, x_indexode(2)) = 0;
		s(4, x_indexode(3)) = 0;
		s(4, x_indexode(4)) = 0;
		s(4, x_indexode(5)) = 1;
		s(4, u_indexode(0)) = 0;

		// dx[5]= -1 * (x[5] - u[0]) / T1;
		s(5, x_indexode(0)) = 0;
		s(5, x_indexode(1)) = 0;
		s(5, x_indexode(2)) = 0;
		s(5, x_indexode(3)) = 0;
		s(5, x_indexode(4)) = 0;
		s(5, x_indexode(5)) = -1 / T1;
		s(5, u_indexode(0)) = 1 / T1;

		// dx[6] = (u[0] * u[0]);
		s(6, x_indexode(0)) = 0;     //!!!!!
		s(6, x_indexode(1)) = 0;
		s(6, x_indexode(2)) = 0;
		s(6, x_indexode(3)) = 0;
		s(6, x_indexode(4)) = 0;
		s(6, x_indexode(5)) = 0;
		s(6, u_indexode(0)) = 2 * u[0];

		return true;
	}
	bool ode_diff_p(DiffStructure& s, double t, const double *x, const double *u, const double *p, int index) {
		return false;
	}



	//--------- Boundaries -----------------------------------------------------//

	void u_boundary(double *u_low, double *u_upp) {
		u_low[0] = lw_bnd[0];
		u_upp[0] = up_bnd[0];
	}

	void x_boundary(double *x_low, double *x_upp) {
		for (i = 0; i < n_ode; i++) {
			x_low[i] = lw_bnd[i+1];
			x_upp[i]  = up_bnd[i+1];
		}
	}
	void p_boundary(double *p_low, double *p_upp) {
	}
	void var_boundary(double *x_low, double *x_upp) {		//where is the difference to the other limit function?
		for (i = 0; i < n_ode; i++) {
			x_low[x_index(0, i)] = start[i];
			x_upp[x_index(0, i)] = start[i];
		}
	}

private:
	int i = 0, j = 0;
};

class MPC_Doppelpendel_real : public TransWorhp {

public:

	MPC_Doppelpendel_real(int dis) : TransWorhp("MPC_Doppelpendel_real", dis, 7, 1, 0, 0, 0) {}

	void GetXTitle(int d, char *s) {
		if (d == 0) strcpy(s, "angle velocity of pendulum 1");
		if (d == 1) strcpy(s, "angle velocity of pendulum 2");
		if (d == 2) strcpy(s, "angle of pendulum 1");
		if (d == 3) strcpy(s, "angle of pendulum 2");
		if (d == 4) strcpy(s, "cart displacement");
		if (d == 5) strcpy(s, "cart velocity");
		if (d == 6) strcpy(s, "input energy");
	}

	void GetUTitle(int d, char *s) {
		if (d == 0) strcpy(s, "input velocity");
	}

	//--------- Start optimization (optional) ---------------------//
	void x_init(double *x, int i, int dis) {
	}
	void u_init(double *u, int i, int dis) {
	}


	//--------- Objective Function -------------------------------------------//
	double obj() {
		return 0;
	}

	bool obj_structure(DiffStructure &s) {
		return false;
	}

	bool obj_diff(DiffStructure &s) {
		return false;
	}

	//--------- System model ------------------------------------------------//

	double det;
	double M11_inv[2][2];
	double M12[2];
	double C1[2];
	double G1[2];
	double N1;
	double N2;
	double def;
	double P;

	void ode(double *dx, double t, const double *x, const double *u, const double *p) {
		//ordinary differential equation(ODE)


		det = (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3]));

		M11_inv[0][0] = h6 / det;
		M11_inv[0][1] = (-1 * h5 * cos(x[2] - x[3])) / det;
		M11_inv[1][0] = (-1 * h5 * cos(x[2] - x[3])) / det;
		M11_inv[1][1] = h4 / det;

		M12[0] = h2 * cos(x[2]);
		M12[1] = h3 * cos(x[3]);

		C1[0] = h5 * x[1] * x[1] * sin(x[2] - x[3]);
		C1[1] = -1 * h5 * x[0] * x[0] * sin(x[2] - x[3]);


		G1[0] = -1 * h7 * sin(x[2]);
		G1[1] = -1 * h8 * sin(x[3]);

		dx[5] = -1 * (x[5] - u[0]) / T1;
		N1 = (M12[0] * dx[5] + C1[0] + G1[0]);
		N2 = (M12[1] * dx[5] + C1[1] + G1[1]);

		dx[0] = -1 * M11_inv[0][0] * N1 - M11_inv[0][1] * N2;
		dx[1] = -1 * M11_inv[1][0] * N1 - M11_inv[1][1] * N2;
		dx[2] = x[0];
		dx[3] = x[1];
		dx[4] = x[5];

		dx[6] = (u[0] * u[0]);
	}


	//--------- Jacobian for Derivatives (Optional) -------------------------//

	bool ode_structure(DiffStructure &s) {
		//return false;
		s(0, x_indexode(0));
		s(0, x_indexode(1));
		s(0, x_indexode(2));
		s(0, x_indexode(3));
		s(0, x_indexode(5));
		s(0, u_indexode(0));

		s(1, x_indexode(0));
		s(1, x_indexode(1));
		s(1, x_indexode(2));
		s(1, x_indexode(3));
		s(1, x_indexode(5));
		s(1, u_indexode(0));

		s(2, x_indexode(0));

		s(3, x_indexode(1));

		s(4, x_indexode(5));

		s(5, x_indexode(5));
		s(5, u_indexode(0));

		s(6, u_indexode(0));

		return true;
	}


	bool ode_diff(DiffStructure& s, double t,  const double* x, const double* u, const double* p) {
			//return false;
			def = (h4 * h6 - h5 * h5 * cos(x[2] - x[3])* cos(x[2] - x[3]));
			P = h5 * h5 * sin(2 * (x[2] - x[3]));

			//dx[0] = -1 * M11_inv[0][0] * N1 - M11_inv[0][1] * N2;
			s(0, x_indexode(0)) = -(2 * x[0] * h5 * h5 * cos(x[2] - x[3]) * sin(x[2] - x[3])) / def;
			s(0, x_indexode(1)) = -(2 * x[1] * h5 * h6 * sin(x[2] - x[3])) / def;
			s(0, x_indexode(2)) = h2*h6*(u[0] - x[5])*(sin(x[2])*def + P*cos(x[2])) / (T1*def*def) - h5*h6*x[1] * x[1] * (cos(x[2] - x[3])*def - P*sin(x[2] - x[3])) / (def*def) + h6*h7*(cos(x[2])*def - P*sin(x[2])) / (def*def) - h3*h5*(u[0] - x[5])*cos(x[3])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def*T1) - h5*h5*x[0] * x[0] * (2 * cos(2 * (x[2] - x[3]))*def - P*sin(2 * (x[2] - x[3]))) / (2 * def*def) + h5*h8*sin(x[3])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def);
		  //s(0, x_indexode(2))=  h2*h6*(u[0] - x[5])*(sin(x[2])*def + P*cos(x[2])) / (T1*def*def) - h5*h6*x[1] * x[1] * (cos(x[2] - x[3])*def - P*sin(x[2] - x[3])) / (def*def) + h6*h7*(cos(x[2])*def - P*sin(x[2])) / (def*def) - h3*h5*(u[0] - x[5])*cos(x[3])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def*T1) - h5*h5*x[0]*x[0]   * (2 * cos(2 * (x[2] - x[3]))*def - P*sin(2 * (x[2] - x[3]))) / (2 * def*def) + h5*h8*sin(x[3])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def);
			s(0, x_indexode(3)) = -h2*h6*cos(x[2])*(u[0] - x[5])*P / (T1*def*def) - h6*h5*x[1] * x[1] * (-cos(x[2] - x[3])*def + P*sin(x[2] - x[3])) / (def*def) + h6*h7*sin(x[2])*P / (def*def) + h5*h3*(u[0] - x[5])*(sin(x[2] - 2 * x[3])*def + P*cos(x[2] - x[3])*cos(x[3])) / (T1*def*def) - h5*h5*x[0] * x[0] * (-2 * cos(2 * (x[2] - x[3]))*def + P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h8*h5*(cos(x[2] - 2 * x[3])*def + P*cos(x[2] - x[3])*sin(x[3])) / (def*def);
		  //s(0, x_indexode(3)) = -h2*h6*cos(x[2])*(u[0] - x[5])*P / (T1*def*def) - h6*h5*x[1] * x[1] * (-cos(x[2] - x[3])*def + P*sin(x[2] - x[3])) / (def*def) + h6*h7*sin(x[2])*P / (def*def) + h5*h3*(u[0] - x[5])*(sin(x[2] - 2 * x[3])*def + P*cos(x[2] - x[3])*cos(x[3])) / (T1*def*def) - h5*h5*x[0] * x[0] * (-2 * cos(2 * (x[2] - x[3]))*def + P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h8*h5*(cos(x[2] - 2 * x[3])*def + P*cos(x[2] - x[3])*sin(x[3])) / (def*def);
			s(0, x_indexode(4)) = 0;
			s(0, x_indexode(5)) = (h2 * h6 * cos(x[2]) - h3 * h5 * cos(x[3]) * cos(x[2] - x[3])) / (T1 * (def));
			s(0, u_indexode(0)) = -(h2 * h6 * cos(x[2]) - h3 * h5 * cos(x[3]) * cos(x[2] - x[3])) / (T1 * (def));

			// dx[1]=dx[1] = -1 * M11_inv[1][0] * N1 - M11_inv[1][1] * N2;
			s(1, x_indexode(0)) = (2 * x[0] * h4 * h5 * sin(x[2] - x[3])) / (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3]));
			s(1, x_indexode(1)) = (2 * x[1] * h5 * h5 * cos(x[2] - x[3]) * sin(x[2] - x[3])) / (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3]));
			s(1, x_indexode(2)) = h5*h2*(u[0] - x[5])*((-cos(x[2] - x[3])*sin(x[2]) - sin(x[2] - x[3])*cos(x[2]))*def - P*cos(x[2])*cos(x[2] - x[3])) / (T1*def*def) + h5*h5*x[1] * x[1] * (2 * cos(2 * (x[2] - x[3]))*def - P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h5*h7*(cos(2 * x[2] - x[3])*def - P*cos(x[2] - x[3])*sin(x[2])) / (def*def) + h4*h3*(u[0] - x[5])*P*cos(x[3]) / (T1*def*def) + h5*h4*x[0] * x[0] * (cos(x[2] - x[3])*def - P*sin(x[2] - x[3])) / (def*def) - h4*h8*sin(x[3])*P / (def*def);
		  //s(1, x_indexode(2)) = h5*h2*(u[0] - x[5])*((-cos(x[2] - x[3])*sin(x[2]) - sin(x[2] - x[3])*cos(x[2]))*def - P*cos(x[2])*cos(x[2] - x[3])) / (T1*def*def) + h5*h5*x[1] * x[1] * (2 * cos(2 * (x[2] - x[3]))*def - P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h5*h7*(cos(2 * x[2] - x[3])*def - P*cos(x[2] - x[3])*sin(x[2])) / (def*def) + h4*h3*(u[0] - x[5])*P*cos(x[3]) / (T1*def*def) + h5*h4*x[0] * x[0] * (cos(x[2] - x[3])*def - P*sin(x[2] - x[3])) / (def*def) - h4*h8*sin(x[3])*P / (def*def);
			s(1, x_indexode(3)) = h2*h5*(u[0] - x[5])*cos(x[2])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def*T1) + h5*h5*x[1] * x[1] * (-2 * cos(2 * (x[2] - x[3]))*def + P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h5*h7*sin(x[2])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def) - h4*h3*(u[0] - x[5])*(-sin(x[3])*def + P*cos(x[3])) / (T1*def*def) + h4*h5*x[0] * x[0] * (-cos(x[2] - x[3])*def + P*sin(x[2] - x[3])) / (def*def) + h4*h8*(cos(x[3])*def + P*sin(x[3])) / (def*def);
		  //s(1, x_indexode(3)) = h2*h5*(u[0] - x[5])*cos(x[2])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def*T1) + h5*h5*x[1] * x[1] * (-2 * cos(2 * (x[2] - x[3]))*def + P*sin(2 * (x[2] - x[3]))) / (2 * def*def) - h5*h7*sin(x[2])*(sin(x[2] - x[3])*def + P*cos(x[2] - x[3])) / (def*def) - h4*h3*(u[0] - x[5])*(-sin(x[3])*def + P*cos(x[3])) / (T1*def*def) + h4*h5*x[0] * x[0] * (-cos(x[2] - x[3])*def + P*sin(x[2] - x[3])) / (def*def) + h4*h8*(cos(x[3])*def + P*sin(x[3])) / (def*def);

			s(1, x_indexode(4)) = 0;
			s(1, x_indexode(5)) = ((h3 * h4 * cos(x[3])) - (h2 * h5 * cos(x[2]) * cos(x[2] - x[3]))) / (T1 * (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3])));
			s(1, u_indexode(0)) = -((h3 * h4 * cos(x[3])) - (h2 * h5 * cos(x[2]) * cos(x[2] - x[3]))) / (T1 * (h4 * h6 - h5 * h5 * cos(x[2] - x[3]) * cos(x[2] - x[3])));

			// dx[2]= x[0];
			s(2, x_indexode(0)) = 1;
			s(2, x_indexode(1)) = 0;
			s(2, x_indexode(2)) = 0;
			s(2, x_indexode(3)) = 0;
			s(2, x_indexode(4)) = 0;
			s(2, x_indexode(5)) = 0;
			s(2, u_indexode(0)) = 0;

			// dx[3]= x[1];
			s(3, x_indexode(0)) = 0;
			s(3, x_indexode(1)) = 1;
			s(3, x_indexode(2)) = 0;
			s(3, x_indexode(3)) = 0;
			s(3, x_indexode(4)) = 0;
			s(3, x_indexode(5)) = 0;
			s(3, u_indexode(0)) = 0;

			// dx[4]=x[5];
			s(4, x_indexode(0)) = 0;
			s(4, x_indexode(1)) = 0;
			s(4, x_indexode(2)) = 0;
			s(4, x_indexode(3)) = 0;
			s(4, x_indexode(4)) = 0;
			s(4, x_indexode(5)) = 1;
			s(4, u_indexode(0)) = 0;

			// dx[5]= -1 * (x[5] - u[0]) / T1;
			s(5, x_indexode(0)) = 0;
			s(5, x_indexode(1)) = 0;
			s(5, x_indexode(2)) = 0;
			s(5, x_indexode(3)) = 0;
			s(5, x_indexode(4)) = 0;
			s(5, x_indexode(5)) = -1 / T1;
			s(5, u_indexode(0)) = 1 / T1;

			// dx[6] = (u[0] * u[0]);
			s(6, x_indexode(0)) = 0;     //!!!!!
			s(6, x_indexode(1)) = 0;
			s(6, x_indexode(2)) = 0;
			s(6, x_indexode(3)) = 0;
			s(6, x_indexode(4)) = 0;
			s(6, x_indexode(5)) = 0;
			s(6, u_indexode(0)) = 2 * u[0];

			return true;
	}
	//*/

	bool ode_diff_p(DiffStructure& s, double t, const double *x, const double *u, const double *p, int index) {
		return false;
	}


	//--------- Boundaries -----------------------------------------------------//

	void u_boundary(double *u_low, double *u_upp) {
	}

	void x_boundary(double *x_low, double *x_upp) {
	}

	void p_boundary(double *p_low, double *p_upp) {
	}

	void var_boundary(double *x_low, double *x_upp) {
	}

private:

	int i = 0, j = 0;
};

/////////////////////////////////////////////////////////////////////////////////
//To calculate with WHORP it will be more fluent if we use this initialize_weights part!

void initialize_weights() {			// here and above at weights check the factors
	 double weightmax = 0;
	for (int i = 0; i <7; i++) {
		max_bnd[i] = (up_bnd[i + 1] > -lw_bnd[i + 1]) ? up_bnd[i + 1] : -lw_bnd[i + 1];
		weightmax += weight[i];
	}
	max_bnd[2] = Pi;		                        
	max_bnd[3] = Pi;
	weightmax += weight[7];
	for (int i = 0; i < 6; i++){
		weight[i] = weight[i] / max_bnd[i] / max_bnd[i]/ weightmax;
	}
	weight[6] = weight[6] / max_bnd[6]/ weightmax;
	gl = 9.81 * a1;
	J_eff = (m1 * l1 * l1) / 3;
	double E2max = m1*g*a1+m2*g*l1+m2*g*a2; // to add mass
	weight[7] = weight[7] / E2max / E2max/ weightmax;
}

int main(int argv, char* argc[]) {

	initialize_weights();

	TWparameter twparameter6("transworhp.xml");
	twparameter6.Arguments(argv, argc);
	twparameter6.NDIS = 6;
	TWfolder folder_simu6(&twparameter6, 0);
	TWfolder folder_real6(&twparameter6, 0);
	TWfolder folder_optim6(&twparameter6, 0);
	TWparameter twparameter5("transworhp.xml");
	twparameter5.Arguments(argv, argc);
	twparameter5.NDIS = 5;
	TWfolder folder_simu5(&twparameter5, 0);
	TWfolder folder_real5(&twparameter5, 0);
	TWfolder folder_optim5(&twparameter5, 0);
	TWparameter twparameter4("transworhp.xml");
	twparameter4.Arguments(argv, argc);
	twparameter4.NDIS = 4;
	TWfolder folder_simu4(&twparameter4, 0);
	TWfolder folder_real4(&twparameter4, 0);
	TWfolder folder_optim4(&twparameter4, 0);
	TWparameter twparameter3("transworhp.xml");
	twparameter3.Arguments(argv, argc);
	twparameter3.NDIS = 3;
	TWfolder folder_simu3(&twparameter3, 0);
	TWfolder folder_real3(&twparameter3, 0);
	TWfolder folder_optim3(&twparameter3, 0);
	TWparameter twparameter2("transworhp.xml");
	twparameter2.Arguments(argv, argc);
	twparameter2.NDIS = 2;
	TWfolder folder_simu2(&twparameter2, 0);
	TWfolder folder_real2(&twparameter2, 0);
	TWfolder folder_optim2(&twparameter2, 0);
	TWparameter twparameter1("transworhp.xml");
	twparameter1.Arguments(argv, argc);
	twparameter1.NDIS = 1;
	TWfolder folder_simu1(&twparameter1, 0);
	TWfolder folder_real1(&twparameter1, 0);
	TWfolder folder_optim1(&twparameter1, 0);
	TWparameter twparameter0("transworhp.xml");
	twparameter0.Arguments(argv, argc);
	twparameter0.NDIS = 0;
	TWfolder folder_simu0(&twparameter0, 0);
	TWfolder folder_real0(&twparameter0, 0);
	TWfolder folder_optim0(&twparameter0, 0);

	MPC_Doppelpendel_optim ph_optim6(6);
	MPC_Doppelpendel_optim ph_simu6(6);
	MPC_Doppelpendel_real ph_real6(6);
	MPC_Doppelpendel_optim ph_optim5(5);
	MPC_Doppelpendel_optim ph_simu5(5);
	MPC_Doppelpendel_real ph_real5(5);
	MPC_Doppelpendel_optim ph_optim4(4);
	MPC_Doppelpendel_optim ph_simu4(4);
	MPC_Doppelpendel_real ph_real4(4);
	MPC_Doppelpendel_optim ph_optim3(3);
	MPC_Doppelpendel_optim ph_simu3(3);
	MPC_Doppelpendel_real ph_real3(3);
	MPC_Doppelpendel_optim ph_optim2(2);
	MPC_Doppelpendel_optim ph_simu2(2);
	MPC_Doppelpendel_real ph_real2(2);
	MPC_Doppelpendel_optim ph_optim1(1);
	MPC_Doppelpendel_optim ph_simu1(1);
	MPC_Doppelpendel_real ph_real1(1);
	MPC_Doppelpendel_optim ph_optim0(0);
	MPC_Doppelpendel_optim ph_simu0(0);
	MPC_Doppelpendel_real ph_real0(0);

	folder_optim6.Add(&ph_optim6);
	folder_optim5.Add(&ph_optim5);
	folder_optim4.Add(&ph_optim4);
	folder_optim3.Add(&ph_optim3);
	folder_optim2.Add(&ph_optim2);
	folder_optim1.Add(&ph_optim1);
	folder_optim0.Add(&ph_optim0);
	folder_real6.Add(&ph_real6);
	folder_real5.Add(&ph_real5);
	folder_real4.Add(&ph_real4);
	folder_real3.Add(&ph_real3);
	folder_real2.Add(&ph_real2);
	folder_real1.Add(&ph_real1);
	folder_real0.Add(&ph_real0);
	folder_simu6.Add(&ph_simu6);
	folder_simu5.Add(&ph_simu5);
	folder_simu4.Add(&ph_simu4);
	folder_simu3.Add(&ph_simu3);
	folder_simu2.Add(&ph_simu2);
	folder_simu1.Add(&ph_simu1);
	folder_simu0.Add(&ph_simu0);
	MPC_Doppelpendel_optim* p_ph_optim[6];
	MPC_Doppelpendel_optim* p_ph_simu[6];
	MPC_Doppelpendel_real* p_ph_real[6];
	TWfolder* p_folder_optim[6];
	TWfolder* p_folder_simu[6];
	TWfolder* p_folder_real[6];
	TWparameter* p_twparameter[6];

	p_ph_optim[5] = &ph_optim6;
	p_ph_optim[4] = &ph_optim5;
	p_ph_optim[3] = &ph_optim4;
	p_ph_optim[2] = &ph_optim3;
	p_ph_optim[1] = &ph_optim2;
	p_ph_optim[0] = &ph_optim1;
	p_ph_simu[5] = &ph_simu6;
	p_ph_simu[4] = &ph_simu5;
	p_ph_simu[3] = &ph_simu4;
	p_ph_simu[2] = &ph_simu3;
	p_ph_simu[1] = &ph_simu2;
	p_ph_simu[0] = &ph_simu1;

	p_ph_real[5] = &ph_real6;
	p_ph_real[4] = &ph_real5;
	p_ph_real[3] = &ph_real4;
	p_ph_real[2] = &ph_real3;
	p_ph_real[1] = &ph_real2;
	p_ph_real[0] = &ph_real1;

	p_folder_optim[5] = &folder_optim6;
	p_folder_optim[4] = &folder_optim5;
	p_folder_optim[3] = &folder_optim4;
	p_folder_optim[2] = &folder_optim3;
	p_folder_optim[1] = &folder_optim2;
	p_folder_optim[0] = &folder_optim1;

	p_folder_simu[5] = &folder_simu6;
	p_folder_simu[4] = &folder_simu5;
	p_folder_simu[3] = &folder_simu4;
	p_folder_simu[2] = &folder_simu3;
	p_folder_simu[1] = &folder_simu2;
	p_folder_simu[0] = &folder_simu1;

	p_folder_real[5] = &folder_real6;
	p_folder_real[4] = &folder_real5;
	p_folder_real[3] = &folder_real4;
	p_folder_real[2] = &folder_real3;
	p_folder_real[1] = &folder_real2;
	p_folder_real[0] = &folder_real1;

	p_twparameter[5] = &twparameter6;
	p_twparameter[4] = &twparameter5;
	p_twparameter[3] = &twparameter4;
	p_twparameter[2] = &twparameter3;
	p_twparameter[1] = &twparameter2;
	p_twparameter[0] = &twparameter1;

	ofstream mpcfile("mpc.dat");
	ofstream mpcfile2("mpc2.dat");
	ofstream mpcfile3("mpc3.dat");
	ofstream mpcfile4("mpc4.dat");

	int state, i, steps, thrsh, thrsh2, temp_n_dis, temp_n_ode, n_dis[3],dummy;

	double base_time_optim, base_time_real, start_time, calc_time_last;
	double calc_time_current, calc_time_estim, sync_time_current, sync_time_estim;
	double calc_time_estim_next, compare_time, temp_time[3], temp_u[6], energy_offset;
	double y_interp[2];

	/*
	TWparameter twparameter100("transworhp.xml");
	twparameter100.Arguments(argv, argc);
	twparameter100.NDIS = 100;
	TWfolder folder_simu100(&twparameter100, 0);
	MPC_Pendel_optim ph_simu100(100);
	folder_simu100.Add(&ph_simu100);
	*/

	try
	{
		Viewer *viewer = nullptr;
		if (p_twparameter[5]->PLOT) {
			std::cout << std::endl;
			viewer = new Viewer(p_twparameter[5]);
		}

		for (temp_n_dis = 2; temp_n_dis < 7; temp_n_dis++) {    //
			for (i = 0; i < temp_n_dis; i++){
			p_ph_simu[temp_n_dis - 1]->T[i] = dis_times[i];
			p_ph_real[temp_n_dis - 1]->T[i] = dis_times[i];
			p_folder_simu[temp_n_dis - 1]->Init();
			p_folder_real[temp_n_dis - 1]->Init();
		}
		}
		for (temp_n_dis = 2; temp_n_dis < 7; temp_n_dis++) {
			for (i = 0; i < temp_n_dis; i++){
				p_ph_optim[temp_n_dis - 1]->T[i] = dis_times[i];
				p_folder_optim[temp_n_dis - 1]->Init();
			}
		}
		n_dis[0] = 6; //optim
		n_dis[1] = 6; //simu
		n_dis[2] = 6; //real

		p_ph_optim[0] = p_ph_optim[n_dis[0] - 1];  //only serves slimmer code

		//??? what does this part mean?
		for (i = 0; i < n_dis[0]; i++) {		//lin Interp. others, MATLAB course, then save, and import, also for control
			control[i] = 0.0;                
			for (state = 0; state < p_ph_optim[0]->n_ode; state++)  //n_ode number of states
				states[i][state] = (start[state] * (dis_times[n_dis[0] - 1] - dis_times[i]) + ziel[state] * dis_times[i]) / dis_times[n_dis[0] - 1];
		}

		p_folder_optim[5]->Init(viewer);
		p_folder_optim[n_dis[0] - 1]->Loop(0,0);
		//loop(wait, terminate).
		calc_time_estim = ext_estim_time;
		sync_time_estim = calc_time_estim;
		base_time_optim = 0.0;
		base_time_real = 0.0;
		calc_time_current = 0.1;

		for (int K = 0; K < 100; K++) { //INFINITY

			start_time = 0.001*(double)GetTickCount();
			// integrate starting detection from initial state and control
			// determine the length of the time vector
			thrsh =1;
			while (dis_times[thrsh] < sync_time_estim - 0.0001)
				thrsh++;
			// store new time vector lines and pointers to a suitable simulation object
			n_dis[1] = thrsh;
			p_ph_simu[0] = p_ph_simu[n_dis[1]]; // only serves slimmer code
			// set initial state
			for (state = 0; state < p_ph_simu[0]->n_ode; state++)
				p_ph_simu[0]->X[p_ph_simu[0]->x_index(0, state)] = start[state];

			// Set control
			for (i = 0; i < thrsh; i++)
				p_ph_simu[0]->X[p_ph_simu[0]->u_index(i, 0)] = p_ph_optim[0]->u(i, 0);

			if (fabs(dis_times[thrsh] - sync_time_estim) > 0.0001)
			{
				//linear interpolation
				y_interp[0] = p_ph_optim[0]->u(thrsh - 1, 0);
				y_interp[1] = p_ph_optim[0]->u(thrsh, 0);
				p_ph_simu[0]->X[p_ph_simu[0]->u_index(thrsh, 0)] =
					interp1(&dis_times[thrsh - 1], y_interp, sync_time_estim);
				p_ph_simu[0]->T[thrsh] = sync_time_estim;
			}
			else
				p_ph_simu[0]->X[p_ph_simu[0]->u_index(thrsh, 0)] = p_ph_optim[0]->u(thrsh, 0);

			//Integrate system model
			steps = p_ph_simu[0]->Integrate(p_twparameter[n_dis[1]]->butchertableau);

			// Time for start-up
			calc_time_current += 0.001*(double)GetTickCount() - start_time;

			/*
			thrsh2 = 0;
			for (i = 0; i < 100; i++) {
			ph_simu100.T[i] = 0.01*sync_time_estim * (double)i;
			if (dis_times[thrsh2] <= ph_simu100.T[i]){
			y_interp[0] = p_ph_optim[0]->u(thrsh2, 0);
			y_interp[1] = p_ph_optim[0]->u(thrsh2 + 1, 0);
			thrsh2++;
			}
			ph_simu100.X[ph_simu100.u_index(i, 0)] =
			interp1(&dis_times[thrsh2-1], y_interp, ph_simu100.T[i]);
			}
			for (state = 0; state < p_ph_simu[0]->n_ode; state++)
			ph_simu100.X[ph_simu100.x_index(0, state)] = start[state];
			ph_simu100.Integrate(twparameter100.butchertableau);
			//*/

			//Hold the controller for the real system
			for (i = 0; i < n_dis[0]; i++)
				temp_u[i] = p_ph_optim[0]->u(i, 0);

			// Textausgabe
			mpcfile << "Unchangeable control signals:" << endl;
			for (i = 0; i <= thrsh; ++i)
				mpcfile << base_time_optim + p_ph_simu[0]->T[i] << " s: "<< p_ph_simu[0]->u(i, 0) << endl;

			mpcfile << endl << "Expected States:";
			for (i = 0; i <= thrsh; i++) {
				mpcfile << endl << base_time_optim + p_ph_simu[0]->T[i] << " s: ";

				mpcfile3 << base_time_optim + p_ph_simu[0]->T[i] << ", " << p_ph_simu[0]->u(i, 0);

				for (state = 0; state <p_ph_simu[0]->n_ode; state++) {
					mpcfile << p_ph_simu[0]->x(i, state) << ", ";
					mpcfile3 << ", " << p_ph_simu[0]->x(i, state);            //simulation part of x(i,state)
				}
				if ((i == 0) && (base_time_real == base_time_optim))
					mpcfile3 << ", 1";
				else
					mpcfile3 << ", 0";

				mpcfile3 << endl;
			}
			mpcfile << endl << endl;
			/*
			for (i = 0; i < 100; i++) {
			mpcfile << base_time_optim + ph_simu100.T[i] << ", " << ph_simu100.u(i, 0);
			for (state = 0; state < p_ph_simu[0]->n_ode; state++) {
			mpcfile << ", " << ph_simu100.x(i, state);
			}
			mpcfile << endl;
			}
			mpcfile << endl;
			//*/

			start_time = 0.001*(double)GetTickCount();

			// Make changes to the bill
			p_ph_simu[0]->T[thrsh] = dis_times[thrsh];

			// optimization

			//Optimization Start values set to simulated  result point
			for (state = 0; state < p_ph_optim[0]->n_ode; state++)
				start[state] = p_ph_simu[0]->x(thrsh, state);

			// take the last run as a start-up optimization
			// Determine energy offset between last planned trajectory and  result value
			temp_n_ode = p_ph_optim[0]->n_ode - 1; //Energie-Index
			y_interp[0] = p_ph_optim[0]->x(thrsh - 1, temp_n_ode);
			y_interp[1] = p_ph_optim[0]->x(thrsh, temp_n_ode);
			energy_offset = start[temp_n_ode] - interp1(&dis_times[thrsh - 1], y_interp, sync_time_estim);

			// take delivery point from Simu
			control[0] = p_ph_simu[0]->u(thrsh, 0);
			for (state = 0; state < p_ph_optim[0]->n_ode; state++)
				states[0][state] = start[state];

			compare_time = sync_time_estim + dis_times[1];
			for (i = 1; compare_time < dis_times[n_dis[0] - 1]; i++) {
				while (dis_times[thrsh] < compare_time) thrsh++;
				temp_time[0] = compare_time - p_ph_optim[0]->T[thrsh - 1];
				temp_time[1] = p_ph_optim[0]->T[thrsh] - compare_time;
				temp_time[2] = p_ph_optim[0]->T[thrsh] - p_ph_optim[0]->T[thrsh - 1];
				control[i] = (p_ph_optim[0]->u(thrsh, 0)*temp_time[0] + p_ph_optim[0]->u(thrsh - 1, 0)*temp_time[1]) / temp_time[2];
				for (state = 0; state < p_ph_optim[0]->n_ode; state++) {
					states[i][state] = (p_ph_optim[0]->x(thrsh, state)*temp_time[0] + p_ph_optim[0]->x(thrsh - 1, state)*temp_time[1]) / temp_time[2];
				}
				states[i][temp_n_ode] += energy_offset; // Energy should start at 0 and otherwise develop the same
				compare_time = sync_time_estim + dis_times[i + 1];
			}

			// above consider less points in time
			temp_n_dis = 3 + (int)fabs(wrap2pi(start[2]));
			temp_n_dis = (temp_n_dis > 3) ? temp_n_dis : 4;
			if ((temp_n_dis < 4) || (temp_n_dis > 6)) {
				mpcfile << base_time_real << "s: Too many or too few elements in the time vector! \ n";
				exit(7);
			}

			//ggf. Longer start-up inspection possible
			for (; i < temp_n_dis; i++) {
				control[i] = control[i - 1];
				for (state = 0; state < p_ph_optim[0]->n_ode; state++) {
					states[i][state] = states[i - 1][state];
				}
			}

			// store new time vectors and pointers to matching optimization object
			n_dis[0] = temp_n_dis;
			p_ph_optim[0] = p_ph_optim[n_dis[0] - 1]; //dient nur schlankerem Code

													  //Perform optimization run
			p_folder_optim[n_dis[0] - 1]->Reinit();
			p_folder_optim[n_dis[0] - 1]->Loop(0, 0);
			//Loop (wait,terminate).
			// Time for start-up + optimization
			calc_time_current += 0.001*(double)GetTickCount() - start_time;

			// lab computer
			mpcfile4 << p_ph_optim[0]->n_dis << ", " << calc_time_current << endl;

			calc_time_current = 0.1;

			//Estimate next time
			calc_time_estim_next =     (calc_time_estim + calc_time_current) * 0.5;
			calc_time_estim_next = max(calc_time_estim_next, calc_time_estim * 0.5);
			calc_time_estim_next = min(calc_time_estim_next, calc_time_estim * 1.5);

			// text output
			mpcfile << "Optimal control:" << endl;
			for (i = 0; i < p_ph_optim[0]->n_dis; i++)
				mpcfile << base_time_optim + calc_time_estim + p_ph_optim[0]->T[i] << " s: " << p_ph_optim[0]->u(i, 0) << endl;
            mpcfile << endl << "States aimed at:";
			 for (i = 0; i < p_ph_optim[0]->n_dis; i++) {
				 mpcfile << endl << base_time_optim + calc_time_estim + p_ph_optim[0]-> T[i] << " s: ";
				mpcfile3 << base_time_optim + calc_time_estim + p_ph_optim[0]->T[i] << ", " << p_ph_optim[0]->u(i, 0);
				for (state = 0; state < p_ph_optim[0]->n_ode; state++) {
					mpcfile <<          p_ph_optim[0]->x(i, state) << ", ";         //  optimal part of x(i,state)
					mpcfile3 << ", " << p_ph_optim[0]->x(i, state);
				}
				if (i == 0)
					mpcfile3 << ", 2";
				else
					mpcfile3 << ", 0";
				mpcfile3 << endl;
			}
			mpcfile << endl << endl;

			//mpcfile4 << calc_time_current << endl;
			//sample_time += calc_time_current;
			//Synchronize times
			sync_time_current = calc_time_current + base_time_optim - base_time_real;
			sync_time_estim   = calc_time_estim   + base_time_optim - base_time_real;

			if (sync_time_estim > dis_times[5]) {
				mpcfile << base_time_real + sync_time_estim << "s: last optim run took too long!\n";
				exit(4);
			}



			// If calculation took less than expected
			// the sequence will not be sent until later,
			// so she fits the condition. The next
			// Calculation can still start when the distance
			// justifies the effort

	if (sync_time_current < sync_time_estim - pre_calc_time)
			{
				calc_time_last = calc_time_current; //caching
				// unter 0 ?
				sync_time_current = max(0.001, sync_time_current);
				// more than the hip shorter than imagined?
				if (sync_time_current < sync_time_estim + 0.001 - calc_time_current)
					mpcfile4 << sync_time_current;
				sync_time_current = max(sync_time_estim + 0.001 - calc_time_current, sync_time_current);
				// integrate real system
				//mpcfile4 << sync_time_estim - sync_time_current << endl;
				// Determine the length of the time vector
				thrsh2 = 0;
				while (dis_times[thrsh2] < sync_time_current) thrsh2++;
				thrsh = thrsh2;
				while (dis_times[thrsh] < sync_time_estim) thrsh++;
				if (thrsh >= n_dis[0]) {
					// Then sync_time_estim is within the max. 6 time steps,
					// but not in the n_dis [0] time steps, those for the last one
					// optimization were used.
					mpcfile << base_time_real + sync_time_estim << "s: last optimization run too short, or calculation now too long => control sequence interrupted! \ n";
					exit(6);
				}
				// store new time vector lines and pointers to a suitable simulation object
				n_dis[2] = thrsh + 2;
				p_ph_real[0] = p_ph_real[n_dis[2] - 1]; // only serves slim code
				//reinitialize the real system
				for (state = 0; state < p_ph_real[0]->n_ode; state++)
					p_ph_real[0]->X[p_ph_real[0]->x_index(0, state)] = start_real[state];

				// Set control, first without special points
				for (i = 0; i < n_dis[2]; i++)
					p_ph_real[0]->X[p_ph_real[0]->u_index(i, 0)] = temp_u[i];
				//linear interpolation  result point
				y_interp[0] = p_ph_real[0]->u(thrsh - 1, 0);
				y_interp[1] = p_ph_real[0]->u(thrsh, 0);
				p_ph_real[0]->X[p_ph_real[0]->u_index(thrsh + 1, 0)] =
					interp1(&dis_times[thrsh - 1], y_interp, sync_time_estim);
				p_ph_real[0]->T[thrsh + 1] = sync_time_estim;
				//Erase values in between
				for (i = thrsh; i > thrsh2; i--) {
					p_ph_real[0]->X[p_ph_real[0]->u_index(i, 0)] = p_ph_real[0]->u(i - 1, 0);
					p_ph_real[0]->T[i] = p_ph_real[0]->T[i - 1];
				}
				//linear interpolation measuring point
				y_interp[0] = p_ph_real[0]->u(thrsh2 - 1, 0);
				y_interp[1] = p_ph_real[0]->u(thrsh2, 0);
				p_ph_real[0]->X[p_ph_real[0]->u_index(thrsh2, 0)] =
					interp1(&dis_times[thrsh2 - 1], y_interp, sync_time_current);
				p_ph_real[0]->T[thrsh2] = sync_time_current;

				//Integrate system
				steps = p_ph_real[0]->Integrate(p_twparameter[n_dis[2] - 1]->butchertableau);

				// text output
				mpcfile << "Control signals done:" << endl;
				for (i = 0; i < thrsh; i++) {
					mpcfile << base_time_real + p_ph_real[0]->T[i] << " s: " << p_ph_real[0]->u(i, 0) << endl;
				}
				mpcfile << endl << "Real states: ";
				for (i = 0; i <= thrsh; i++) {
					mpcfile << endl << base_time_real + p_ph_real[0]->T[i] << " s: ";
					mpcfile2 << base_time_real + p_ph_real[0]->T[i] << ", " << p_ph_real[0]->u(i, 0) << ", ";
					for (state = 0; state < p_ph_real[0]->n_ode; state++) {
						mpcfile << p_ph_real[0]->x(i, state) << ", ";
						mpcfile2 << p_ph_real[0]->x(i, state) << ", ";
					}
					mpcfile2 << "1" << endl;                         // nomatterwhat value i changed  the result does no changes 
				}
				mpcfile << endl << endl;

				// undo changes
				for (i = thrsh2; i < n_dis[2]; i++) {
					p_ph_real[0]->X[p_ph_real[0]->u_index(i, 0)] = temp_u[i];
					p_ph_real[0]->T[i] = dis_times[i];
				}

				// Fest Hold down the result point for the real system
				for (state = 0; state < p_ph_real[0]->n_ode - 1; state++) {
					start_real[state] = p_ph_real[0]->x(thrsh, state);
				}
				start_real[p_ph_real[0]->n_ode - 1] = 0.0; //Start energy again at 0

				// Start detection from initial state and integrate control
				start_time = 0.001*(double)GetTickCount();

				//Determine the length of the time vector
				thrsh = 1;
				while (dis_times[thrsh] < sync_time_current) thrsh++;
				i = 0;
				while (dis_times[thrsh + i] < sync_time_estim) i++;
				// store new time vector lines and pointers to matching simulation objects
				n_dis[1] = i + 2;
				p_ph_simu[0] = p_ph_simu[n_dis[1] - 1]; //only serves slimmer code

				// Initial state Determine start detection
				p_ph_simu[0]->X[p_ph_simu[0]->x_index(0, 0)] = p_ph_real[0]->x(thrsh2, 0);
				p_ph_simu[0]->X[p_ph_simu[0]->x_index(0, 1)] = p_ph_real[0]->x(thrsh2, 1);
				p_ph_simu[0]->X[p_ph_simu[0]->x_index(0, 2)] = p_ph_real[0]->x(thrsh2, 3);
				p_ph_simu[0]->X[p_ph_simu[0]->x_index(0, 3)] = p_ph_real[0]->x(thrsh2, 4);
				p_ph_simu[0]->X[p_ph_simu[0]->x_index(0, 4)] = 0.0; //Start energy again at 0

				// set control
				// linear interpolation for the measuring point
				y_interp[0] = p_ph_real[0]->u(thrsh - 1, 0);
				y_interp[1] = p_ph_real[0]->u(thrsh, 0);
				p_ph_simu[0]->X[p_ph_simu[0]->u_index(0, 0)] =
					interp1(&dis_times[thrsh - 1], y_interp, sync_time_current);
				//Values in between
				for (thrsh2 = thrsh; thrsh2 < thrsh + n_dis[1] - 2; thrsh2++) {
					i = thrsh2 - thrsh + 1;
					p_ph_simu[0]->X[p_ph_simu[0]->u_index(i, 0)] = p_ph_real[0]->u(thrsh2, 0);
					p_ph_simu[0]->T[i] = dis_times[thrsh2] - sync_time_current;
				}
				//linear interpolation for the result point
				y_interp[0] = p_ph_real[0]->u(thrsh2 - 1, 0);
				y_interp[1] = p_ph_real[0]->u(thrsh2, 0);
				p_ph_simu[0]->X[p_ph_simu[0]->u_index(n_dis[1] - 1, 0)] =
					interp1(&dis_times[thrsh2 - 1], y_interp, sync_time_estim);
				p_ph_simu[0]->T[n_dis[1] - 1] = sync_time_estim - sync_time_current;

				// Integrate system
				steps = p_ph_simu[0]->Integrate(p_twparameter[n_dis[1] - 1]->butchertableau);

				//Integral as a new start value
				for (state = 0; state < p_ph_simu[0]->n_ode; state++)
					start[state] = p_ph_simu[0]->x(thrsh, state);

				calc_time_current = 0.001*(double)GetTickCount() - start_time;

					//text output
				mpcfile << "Unchangeable control signals:" << endl;
				for (i = 0; i < thrsh; i++)
					mpcfile << base_time_real + p_ph_simu[0]->T[i] << " s: " << p_ph_simu[0]->u(i, 0) << endl;
				mpcfile << endl << "Expected States:";
				for (i = 0; i <= thrsh; i++) {
					mpcfile << endl << base_time_optim + p_ph_simu[0]->T[i] << " s: ";
					mpcfile3 << base_time_optim + p_ph_simu[0]->T[i] << ", " << p_ph_simu[0]->u(i, 0);
					for (state = 0; state < p_ph_simu[0]->n_ode; state++) {
						mpcfile << p_ph_simu[0]->x(i, state) << ", ";

						mpcfile3 << ", " << p_ph_simu[0]->x(i, state);
					}
					if (i == 0)
						mpcfile3 << ", 3";
					else
						mpcfile3 << ", 0";
					mpcfile3 << endl;
				}
				mpcfile << endl << endl;
				// Undo changes
				for (i = 0; i < n_dis[1]; i++)
					p_ph_simu[0]->T[i] = dis_times[i];

				// Continue timestamp
				base_time_real += sync_time_estim;
				base_time_optim += calc_time_last;
			}



	else
			    	// at less than pre_calc_time remaining time, exactly hit or
				    // too long calculation of control sequence
			{		
				// integrate real system 

				// transfer time
				sync_time_current = max(sync_time_current, sync_time_estim);

				// Determine the length of the time vector
				thrsh = 1;
				while ( dis_times[thrsh] < sync_time_current - 0.0001 )
					thrsh++;
				//Check if optimization vector was sufficient
				if (thrsh >= n_dis[0]) {
					// Then sync_time_estim is within the max. 6 time steps,
					// but not in the n_dis [0] time steps, those for the last one
					// optimization were used.
					mpcfile << base_time_real + sync_time_estim << "s: last optimization run too short, or calculation now too long => control sequence interrupted! \ n";
					exit(6);
				}
				// store new time vector lines and pointers to a suitable simulation object

				n_dis[2] = thrsh;

				 p_ph_real[0] = p_ph_real[n_dis[2]]; // only serves slimmer code

			// reinitialize the real system
			for (state = 0; state < p_ph_real[0]->n_ode; state++)
					   p_ph_real[0]->X[p_ph_real[0]->x_index(0, state)] = start_real[state];

			// Set control
			for (i = 0; i < n_dis[2]; i++)
				p_ph_real[0]->X[p_ph_real[0]->u_index(i, 0)] = temp_u[i];
		
			if (fabs(dis_times[thrsh] - sync_time_current) > 0.0001)
			{
				//linear interpolation
				y_interp[0] = p_ph_real[0]->u(thrsh - 1, 0);
				y_interp[1] = p_ph_real[0]->u(thrsh, 0);
				p_ph_real[0]->X[p_ph_real[0]->u_index(thrsh, 0)] =
					interp1(&dis_times[thrsh - 1], y_interp, sync_time_current);
				p_ph_real[0]->T[thrsh] = sync_time_current;
			}
			else
				p_ph_real[0]->X[p_ph_real[0]->u_index(thrsh, 0)] = temp_u[thrsh];

				//Integrate system
				steps = p_ph_real[0]->Integrate(p_twparameter[n_dis[2]]->butchertableau);

				// text output
				mpcfile << "Detailed control signals:" << endl;
				for (i = 0; i < thrsh; i++) {
					mpcfile << base_time_real + p_ph_real[0]->T[i] << " s: " << p_ph_real[0]->u(i, 0) << endl;
				}
				mpcfile << endl << "Real condition: ";
				for (i = 0; i <= thrsh; i++) {
					mpcfile << endl << base_time_real + p_ph_real[0]->T[i] << " s: ";
					mpcfile2 << base_time_real + p_ph_real[0]->T[i] << ", " << p_ph_real[0]->u(i, 0) << ", ";
					for (state = 0; state < p_ph_real[0]->n_ode; state++) {
						mpcfile  << p_ph_real[0]->x(i, state) << ", ";
						mpcfile2 << p_ph_real[0]->x(i, state) << ", ";
					}
					mpcfile2 << "0" << endl;
				}
				mpcfile << endl << endl;

				// Undo changes
				p_ph_real[0]->T[thrsh] = dis_times[thrsh];

				// measurement signal after start estimation + optimization as new start value,
				//  Hold down the result of the real system
				for (state = 0; state < p_ph_real[0]->n_ode - 1; state++) {
					start[state] = p_ph_real[0]->x(thrsh, state);
					start_real[state] = start[state];
				}
				start[p_ph_simu[0]->n_ode - 1] = 0.0;
				start_real[p_ph_real[0]->n_ode - 1] = 0.0; //Energie wieder bei 0 starten
														   //Start energy again at 0

				// Continue total time
				base_time_real += sync_time_current;
				base_time_optim = base_time_real;

			}
			calc_time_current = 0.1;
			// Average expected time for start-up + optimization
			calc_time_estim = calc_time_estim_next;                //???
		}
		mpcfile.close();
		mpcfile2.close();
		mpcfile3.close();
		mpcfile4.close();
	}
	catch (int e)
	{
		printf("An error accured: error = %d\n", e);
		printf("Application will stop!");
		printf("PRESS ANY KEY!\r\n");
		while (!_kbhit());
		_getch();
		exit(1);
	}

	while (!_kbhit());
	_getch();
	return 0;
}
