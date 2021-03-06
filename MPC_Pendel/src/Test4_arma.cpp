/*----------------------------------------------------------------
*
* MPC_Pendel: Test-MPC-Regler f�r das inverse Pendel
*
*----------------------------------------------------------------*/

#define NOMINMAX

#ifdef WIN32
#include "windows.h"
#endif

//#define NOGRAPHICS -> Buggy!

#include <conio.h>
#include <armadillo>
#include "TransWORHP.h"

using namespace std;
using namespace arma;

const double Pi = 3.1415926535898;
const double Pi2 = 6.2831853071796;

vec start, ziel, weight, up_bnd, lw_bnd, dis_times, max_bnd;

int error;

double gl, l, J_eff;


class MPC_Pendel_optim : public TransWorhp {

public:

	MPC_Pendel_optim(int dis) : TransWorhp("MPC_Pendel_optim", dis, 5, 1, 0, 0, 0) {}

	void GetXTitle(int d, char *s) {
		if (d == 0) strcpy(s, "position x of the cart");
		if (d == 1) strcpy(s, "cart velocity");
		if (d == 2) strcpy(s, "pendulum angle ");
		if (d == 3) strcpy(s, "pendulum angular velocity ");
		if (d == 4) strcpy(s, "Energy");
	}

	void GetUTitle(int d, char *s) {
		if (d == 0) strcpy(s, "input velocity");
	}


	//--------- Startsch�tzung der Optimierung (optional) ---------------------//

	void x_init(double *x, int i, int dis) {
		for (j = 0; j < n_ode; j++) {
			x[j] = (start(j)*(dis - i) + ziel(j)*i) / dis;
		}
	}
	void u_init(double *u, int i, int dis) {
		u[0] = 0.0;
	}


	//--------- Objective Function -------------------------------------------//

	double obj() {

		double tmp, ret = 0;
		for (j = n_dis - 3; j < n_dis; j++) {
			tmp = (x(j, 0) - ziel(0));
			ret += tmp*tmp * weight(0);

			tmp = (x(j, 1) - ziel(1));
			ret += tmp*tmp * weight(1);

			tmp = fmod(x(j, 2) - ziel(2) + Pi, Pi2) - Pi;
			ret += tmp*tmp * weight(2);

			tmp = (x(j, 3) - ziel(3));
			ret += tmp*tmp * weight(3);

			ret += x(j, 4) * weight(4);

			tmp = cos(x(j, 2)) - cos(ziel(2));
			tmp *= x(j, 1)*x(j, 3)*l + gl;
			tmp += x(j, 3)*x(j, 3)*J_eff;
			tmp += x(j, 1)*x(j, 1);
			ret += tmp*tmp* weight(5);
		}
		return ret;
	}

	bool obj_structure(DiffStructure &s) {
		return false;
		s(0, x_index(n_dis - 1, 4));
		return true;
	}

	bool obj_diff(DiffStructure &s) {
		return false;
		s(0, x_index(n_dis - 1, 4)) = 1;
		return true;
	}

	//--------- System model ------------------------------------------------//

	void ode(double *dx, double t, const double *x, const double *u, const double *p) {
		double  T1 = 0.0395;
		double  c = (6 / ((4 * 0.5) + ((0.02*0.02) / 0.5)));
		double  g = 9.81;

		dx[0] = x[1];
		dx[1] = (u[0] - x[1]) / T1;
		dx[2] = x[3];
		dx[3] = -1 * (((u[0] - x[1]) / T1)*c*cos(x[2])) + (c*g*sin(x[2])) - (0.01*c*x[3]);
		dx[4] = (u[0] * u[0]);
	}


	//--------- Jacobian for Derivatives (Optional) -------------------------//

	bool ode_structure(DiffStructure &s) {
		s(0, x_indexode(1));

		s(1, x_indexode(1));
		s(1, u_indexode(0));

		s(2, x_indexode(3));

		s(3, x_indexode(1));
		s(3, x_indexode(2));
		s(3, x_indexode(3));
		s(3, u_indexode(0));

		s(4, u_indexode(0));

		return true;
	}

	bool ode_diff(DiffStructure& s, double t, const double* x, const double* u, const double* p) {
		double  T1 = 0.0395;
		double  c = (6 / ((4 * 0.5) + ((0.02*0.02) / 0.5)));
		double  g = 9.81;

		//dx[0] = dx[0] = x[1];
		s(0, x_indexode(0)) = 0;
		s(0, x_indexode(1)) = 1;
		s(0, x_indexode(2)) = 0;
		s(0, x_indexode(3)) = 0;
		s(0, u_indexode(0)) = 0;

		// dx[1] = (u[0] - x[1]) / T1;
		s(1, x_indexode(0)) = 0;
		s(1, x_indexode(1)) = -1 / T1;
		s(1, x_indexode(2)) = 0;
		s(1, x_indexode(3)) = 0;
		s(1, u_indexode(0)) = 1 / T1;

		// dx[2] = x[3];
		s(2, x_indexode(0)) = 0;
		s(2, x_indexode(1)) = 0;
		s(2, x_indexode(2)) = 0;
		s(2, x_indexode(3)) = 1;
		s(2, u_indexode(0)) = 0;

		// dx[3] = (((u[0] - x[1]) / T1)*c*cos(x[2])) + (c*g*sin(x[2])) - (0.01*c*x[3]);
		s(3, x_indexode(0)) = 0;
		s(3, x_indexode(1)) = 0;
		s(3, x_indexode(2)) = c*g*cos(x[2]);
		s(3, x_indexode(3)) = -0.01*c;
		s(3, u_indexode(0)) = 0;

		// dx[4] = (u[0] * u[0]);
		s(4, x_indexode(0)) = 0;
		s(4, x_indexode(1)) = 0;
		s(4, x_indexode(2)) = 0;
		s(4, x_indexode(3)) = 0;
		s(4, u_indexode(0)) = 1;

		return true;
	}

	bool ode_diff_p(DiffStructure& s, double t, const double *x, const double *u, const double *p, int index) {
		return false;
	}


	//--------- Boundaries -----------------------------------------------------//

	void u_boundary(double *u_low, double *u_upp) {
		u_low[0] = lw_bnd(0);
		u_upp[0] = up_bnd(0);
	}

	void x_boundary(double *x_low, double *x_upp) {
		for (i = 0; i < n_ode; i++) {
			x_low[i] = lw_bnd(i + 1);
			x_upp[i] = up_bnd(i + 1);
		}
	}


	void p_boundary(double *p_low, double *p_upp) {
	}

	void var_boundary(double *x_low, double *x_upp) {
		for (i = 0; i<n_ode; i++) {
			x_low[x_index(0, i)] = start(i);
			x_upp[x_index(0, i)] = start(i);
		}
	}

private:

	int i = 0, j = 0;
};


/////////////////////////////////////////////////////////////////////////////


void initialize_weights() {
	double weightmax = 0;
	max_bnd = max(up_bnd.subvec(1, 5), -lw_bnd.subvec(1, 5));
	max_bnd(2) = Pi;
	weightmax = sum(weight);

	weight /= weightmax;
	weight.subvec(0, 3) /= square(max_bnd.subvec(0, 3));
	weight(4) /= max_bnd(4);

	l = 0.5;
	gl = 9.81*l;
	J_eff = (4.0*l*l + 0.02*0.02) / 12.0;
	//double E2max = max_bnd[1] * max_bnd[1] + J_eff*max_bnd[3] * max_bnd[3];
	//E2max = (E2max >(gl*2.0)) ? E2max : gl*2.0;

	weight(5) /= (gl*gl*6.25);
}


int main(int argv, char* argc[]) {
	
	double alltime = 0.001*(double)GetTickCount();

	start << 0 << 0 << Pi << 0 << 0;
	ziel << 0 << 0 << 0 << 0 << 0;
	weight << 0.5 << 0.0 << 1.0 << 0.0 << 0.02 << 1.0;
	// Wagenlage, Wagengeschw., Pendellage, Pendelgeschw., Stellenergie, Pendelenergie
	// werden jeweils auf Grenze, bzw. Pi, normiert 
	up_bnd << 1.2 << 0.38 << 1.2 << 1e20 << 25 << 0.5;
	lw_bnd << -1.2 << -0.38 << -1.2 << -1e20 << -25 << 0;
	// Sollgeschw., Wagenlage, Wagengeschw., Pendellage, Pendelgeschw., Stellenergie

	dis_times << 0.0 << 0.04 << 0.08 << 0.12 << 0.2 << 0.7;

	initialize_weights();

	TWparameter twparameter("transworhp.xml");
	twparameter.Arguments(argv, argc);

	twparameter.NDIS = 5;
	TWfolder folder_optim(&twparameter, 0);

	MPC_Pendel_optim ph_optim(5);

	folder_optim.Add(&ph_optim);

	ofstream mpcfile("mpc.dat");

	int state, i, steps, thrsh;
	
	double time, mu, sigma;
	double times[20];

	try
	{
		for (i = 0; i < ph_optim.n_dis; i++)
			ph_optim.T[i] = dis_times[i];

		folder_optim.Init();

		mpcfile << "jeweils Mittelwert, Streuung" << endl;
		mpcfile << "geschachtelte Schleifen:" << endl;
		mpcfile << "alpha_pkt: -2:1:2, Zeilensprung" << endl;
		mpcfile << "alpha: -Pi:Pi/4:Pi-Pi/4" << endl;
		mpcfile << "x_pkt: -0.5:0.25:0.5" << endl;
		mpcfile << "x: 0:0.1:0.3" << endl << endl;

		for (double x = 0; x < 0.4; x+=0.1) {

			start[0] = x;

			for (double x_pkt = -0.5; x_pkt <= 0.5; x_pkt += 0.25) {

				start[1] = x_pkt;

				for (double alpha = -Pi; alpha < Pi; alpha += (Pi / 4)) {

					start[2] = alpha;

					for (double alpha_pkt = -2.0; alpha_pkt <= 2.0; alpha_pkt += 1.0) {

						start[3] = alpha_pkt;

						mu = 0.0;
						sigma = 0.0;

						for (int K = 0; K < 20; K++) {
							time = 0.001*(double)GetTickCount();

							folder_optim.Reinit();
							folder_optim.Loop(1, 0);

							times[K] = 0.001*(double)GetTickCount() - time;
							mu += times[K];
						}
						mu *= 0.05; // 1/20

						for (int K = 0; K < 20; K++) {
							time = times[K] - mu;
							sigma += (time*time);
						}
						sigma *= 0.052631578947368; // 1/19
						sigma = sqrt(sigma);

						mpcfile << mu << ", " << sigma << ", ";
					}
					mpcfile << endl;
				}
			}
		}
		alltime = 0.001*(double)GetTickCount() - alltime;
		mpcfile << alltime;
		mpcfile.close();
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

	return 0;
}
