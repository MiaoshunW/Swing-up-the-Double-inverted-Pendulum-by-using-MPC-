/*----------------------------------------------------------------
*
* MPC_Pendel: Test-MPC-Regler für das inverse Pendel
*
*----------------------------------------------------------------*/

#ifdef WIN32
#include "windows.h"
#endif

//#define NOGRAPHICS -> Buggy!

#include <conio.h>
#include "TransWORHP.h"

using namespace std;

const double Pi  = 3.1415926535898;
const double Pi2 = 6.2831853071796;

double start[] = { 0, 0, Pi, 0, 0 };
double ziel[] = { 0, 0, 0, 0, 0 };

double weight[] = { 0.5, 0.0, 1.0, 0.0, 0.1, 1.0 };
// Wagenlage, Wagengeschw., Pendellage, Pendelgeschw., Stellenergie, Pendelenergie
// werden jeweils auf Grenze, bzw. Pi, normiert 
const double up_bnd[] = { 0.5, 0.38, 0.5, 1e20, 25, 0.1 };
const double lw_bnd[] = { -0.5, -0.38, -0.5, -1e20, -25, 0 };
// Sollgeschw., Wagenlage, Wagengeschw., Pendellage, Pendelgeschw., Stellenergie

//double dis_times[] = { 0.0, 0.04, 0.1, 0.2, 0.5, 1.0};
//double dis_times[] = { 0.0, 0.04, 0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32 }; //MOD
double dis_times[] = { 0.0, 0.04, 0.08, 0.12, 0.2, 0.7 };

int error;

double gl,l,J_eff;
double max_bnd[5];


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


	//--------- Startschätzung der Optimierung (optional) ---------------------//

	void x_init(double *x, int i, int dis) {
		for (j = 0; j < n_ode; j++) {
			x[j] = (start[j]*(dis-i)+ziel[j]*i)/dis;
		}
	}
	void u_init(double *u, int i, int dis) {
		u[0] = 0.0;
	}


	//--------- Objective Function -------------------------------------------//

	double obj() {

		double tmp, ret = 0;
		for (j = n_dis - 3; j < n_dis; j++) {
			tmp = (x(j, 0) - ziel[0]);
			ret += tmp*tmp * weight[0];

			tmp = (x(j, 1) - ziel[1]);
			ret += tmp*tmp * weight[1];

			tmp = fmod(x(j, 2) - ziel[2] + Pi, Pi2) - Pi;
			ret += tmp*tmp * weight[2];

			tmp = (x(j, 3) - ziel[3]);
			ret += tmp*tmp * weight[3];

			ret += x(j, 4) * weight[4];

			tmp = cos(x(j, 2)) - cos(ziel[2]);
			tmp *= x(j, 1)*x(j, 3)*l + gl;
			tmp += x(j, 3)*x(j, 3)*J_eff;
			tmp += x(j, 1)*x(j, 1);
			ret += tmp*tmp* weight[5];
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
		u_low[0] = lw_bnd[0];
		u_upp[0] = up_bnd[0];
	}

	void x_boundary(double *x_low, double *x_upp) {
		for (i = 0; i < n_ode; i++) {
			x_low[i] = lw_bnd[i + 1];
			x_upp[i] = up_bnd[i + 1];
		}
	}


	void p_boundary(double *p_low, double *p_upp) {
	}

	void var_boundary(double *x_low, double *x_upp) {
		for (i = 0; i<n_ode; i++) {
			x_low[x_index(0, i)] = start[i];
			x_upp[x_index(0, i)] = start[i];
		}
	}

private:

	int i = 0, j = 0;
};

class MPC_Pendel_real : public TransWorhp {

public:

	MPC_Pendel_real(int dis) : TransWorhp("MPC_Pendel_real", dis, 5, 1, 0, 0, 0) {}

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


	//--------- Startschätzung der Optimierung (optional) ---------------------//
	void x_init(double *x, int i, int dis) {
		for (j = 0; j < n_ode; j++) {
			x[j] = (start[j] * (dis - i) + ziel[j] * i) / dis;
		}
	}
	void u_init(double *u, int i, int dis) {
		u[0] = 0;
	}


	//--------- Objective Function -------------------------------------------//
	double obj() {
		double ret = 0;
		for (i = 0; i < n_ode; i++) {
			double tmp = (x(n_dis - 1, i) - ziel[i]);
			ret += tmp*tmp* weight[i];
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
		double  T1 = 0.025;
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
		double  T1 = 0.025;
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
		u_low[0] = lw_bnd[0];
		u_upp[0] = up_bnd[0];
	}

	void x_boundary(double *x_low, double *x_upp) {
		for (i = 0; i < n_ode; i++) {
			x_low[i] = lw_bnd[i+1];
			x_upp[i] = up_bnd[i+1];
		}
	}

	void p_boundary(double *p_low, double *p_upp) {
	}

	void var_boundary(double *x_low, double *x_upp) {
		for (i = 0; i < n_ode; i++) {
			x_low[x_index(0, i)] = start[i];
			x_upp[x_index(0, i)] = start[i];
		}
	}

private:

	int i = 0, j = 0; 
};


/////////////////////////////////////////////////////////////////////////////


void Warmstart(TWfolder &folder) {
	folder.worhp_w.Calls = 0;
	folder.worhp_w.MajorIter = 0;
	folder.worhp_w.MinorIter = 0;
	//	worhp_w.ScaledKKT = 1000;

	folder.worhp_c.status = Iterating;
	SetNextStage(&folder.worhp_c, Pre_KKT);

	folder.worhp_w.ArmijoAlpha = 0;
	folder.worhp_w.Norm2_DX = 0;

	folder.worhp_w.BettsTau = 1e-4;

	for (TransWorhp* ph : folder.phases) {
		for (int i = 0; i < ph->n_dis; i++) {
			ph->X[ph->x_index(i, 4)] = 0;
		}
	}

	for (int i = 0; i < folder.worhp_o.n; i++) {
		folder.worhp_o.Lambda[i] = 0;
	}
	for (int i = 0; i < folder.worhp_o.m; i++) {
		folder.worhp_o.Mu[i] = 0;
	}

	folder.worhp_o.F = folder.worhp_w.ScaleObj * 1;
}

void initialize_weights() {
	double weightmax = 0;
	for (int i = 0; i < 5; i++) {
		max_bnd[i] = (up_bnd[i + 1] > -lw_bnd[i + 1]) ? up_bnd[i + 1] : -lw_bnd[i + 1];
		weightmax += weight[i];
	}
	max_bnd[2] = Pi;
	weightmax += weight[5];

	for (int i = 0; i < 4; i++)
		weight[i] = weight[i] / max_bnd[i] / max_bnd[i] / weightmax;

	weight[4] = weight[4] / max_bnd[4] / weightmax;

	l = 0.5;
	gl = 9.81*l;
	J_eff = (4.0*l*l + 0.02*0.02) / 12.0;
	//double E2max = max_bnd[1] * max_bnd[1] + J_eff*max_bnd[3] * max_bnd[3];
	//E2max = (E2max >(gl*2.0)) ? E2max : gl*2.0;
	double E2max = gl*2.5;

	weight[5] = weight[5] / E2max / E2max / weightmax;
}


int main(int argv, char* argc[]) {

	initialize_weights();

	TWparameter twparameter("transworhp.xml");
	twparameter.Arguments(argv, argc);

	twparameter.NDIS = 6; //6
	TWfolder folder_optim(&twparameter, 0);
	twparameter.NDIS = 2;
	TWfolder folder_real(&twparameter, 0);

	MPC_Pendel_optim ph_optim(6);  //6
	MPC_Pendel_real ph_real(2);

	folder_optim.Add(&ph_optim);
	folder_real.Add(&ph_real);

	ofstream mpcfile("mpc.dat");
	ofstream mpcfile2("mpc2.dat");
	ofstream mpcfile3("mpc3.dat");

	int state, i, steps, thrsh;
	
	double control[6];
	//double x[4];

	double time, time1, time2, delta, tmp, tmp2;

	try
	{
		for (i = 0; i < ph_optim.n_dis; i++)
			ph_optim.T[i] = dis_times[i];
		for (i = 0; i < ph_real.n_dis; i++)
			ph_real.T[i] = 0.001*(double)i;

		folder_optim.Init();

		time1 = 0.001*(double)GetTickCount();

		ph_optim.n_dis = 5;
		folder_optim.Loop(1, 0);
		ph_optim.n_dis = 6;

		time2 = 0.001*(double)GetTickCount();

		/*
		//MOD:
		time = 0.0; delta = 0.0;
		mpcfile << "Optimale Steuerung: " << endl;
		for (i = 0; i < ph_optim.n_dis; i++)
			mpcfile << time + delta + ph_optim.T[i] << " s: " << ph_optim.u(i, 0) << endl;
		mpcfile << endl << "Angestrebte Zustände: ";
		for (i = 0; i < ph_optim.n_dis; i++) {
			mpcfile << endl << time + delta + ph_optim.T[i] << " s: ";
			for (state = 0; state < ph_optim.n_ode; state++)
				mpcfile << ph_optim.x(i, state) << ", ";
		}
		mpcfile << endl << endl;
		//MOD Ende
		*/

		delta = time2 - time1;
		//mpcfile2 << delta << endl;
		//delta = 0.05; //MOD, LÖSCHEN
		time = 0.0;

		folder_real.Init();

		for (int K = 0; K < 60; K++) { //INFINITY
						
			// Steuerfolge für echtes Pendel zwischenspeichern
			for (i = 0; i < ph_optim.n_dis; i++)
				control[i] = ph_optim.u(i, 0);
			time1 = 0.001*(double)GetTickCount();

			// Startschätzung aus Anfangszustand und Steuerung integrieren

				// Anfangszustand festlegen
				for (state = 0; state < ph_optim.n_ode; state++)
					ph_optim.X[ph_optim.x_index(0, state)] = start[state];

				// Steuerung festlegen
				thrsh = 1;
				while (dis_times[thrsh] < delta) thrsh++;
				if (thrsh >= ph_optim.n_dis) {
					mpcfile << delta << "s: last optim run took too long!\n";
					exit(2);
				}
				//lineare Interpolation
				ph_optim.X[ph_optim.u_index(thrsh, 0)] = (ph_optim.u(thrsh, 0)*(delta - ph_optim.T[thrsh - 1])
					+ ph_optim.u(thrsh - 1, 0)*(ph_optim.T[thrsh] - delta)) / (ph_optim.T[thrsh] - ph_optim.T[thrsh - 1]);
				ph_optim.T[thrsh] = delta;
				ph_optim.n_dis = thrsh+1;

				// Systemmodell integrieren
				steps = ph_optim.Integrate(twparameter.butchertableau);

			// Zeit für Startschätzung
			time2 = 0.001*(double)GetTickCount() - time1;

			// Textausgabe
				mpcfile << "Unabänderliche Steuersignale: " << endl;
				for (i = 0; i < thrsh; i++)
					mpcfile << time + ph_optim.T[i] << " s: " << ph_optim.u(i, 0) << endl;
				mpcfile << endl << "Erwartete Zustände: ";
				for (i = 0; i <= thrsh; i++) {
					mpcfile << endl << time + ph_optim.T[i] << " s: ";
					mpcfile3 << time + ph_optim.T[i] << ", " << ph_optim.u(i, 0);
					for (state = 0; state < ph_optim.n_ode; state++) {
						mpcfile << ph_optim.x(i, state) << ", ";
						mpcfile3 << ", " << ph_optim.x(i, state);
					}
					mpcfile3 << endl;
				}
				mpcfile << endl << endl;

			time1 = 0.001*(double)GetTickCount();

			// Änderungen rückgängig machen
				ph_optim.T[thrsh] = dis_times[thrsh];
				ph_optim.n_dis = 6;

			// Optimierung initialisieren mit Startschätzung
				for (state = 0; state < ph_optim.n_ode; state++)
					start[state] = ph_optim.x(thrsh, state);

			// Optimierung
				//Warmstart(folder_optim);
				folder_optim.Reinit();
				ph_optim.n_dis = 3 + (int)fabs(fmod(start[2] + Pi, Pi2) - Pi);
				ph_optim.n_dis = (ph_optim.n_dis > 3) ? ph_optim.n_dis : 4;
				ph_optim.n_dis = (ph_optim.n_dis < 6) ? ph_optim.n_dis : 5;
				folder_optim.Loop(1, 0);

			// Zeit für Startschätzung + Optimierung
			time2 += 0.001*(double)GetTickCount() - time1;
//			mpcfile2 << time2 << endl;

			// Textausgabe
				mpcfile << "Optimale Steuerung: " << endl;
				for (i = 0; i < ph_optim.n_dis; i++)
					mpcfile << time + delta + ph_optim.T[i] << " s: " << ph_optim.u(i, 0) << endl;
				mpcfile << endl << "Angestrebte Zustände: ";
				for (i = 0; i < ph_optim.n_dis; i++) {
					mpcfile << endl << time + delta + ph_optim.T[i] << " s: ";
					mpcfile3 << time + delta + ph_optim.T[i] << ", " << ph_optim.u(i, 0);
					for (state = 0; state < ph_optim.n_ode; state++) {
						mpcfile << ph_optim.x(i, state) << ", ";
						mpcfile3 << ", " << ph_optim.x(i, state);
					}
					mpcfile3 << endl;
				}
				mpcfile << endl << endl;

				ph_optim.n_dis = 6;

			// Wenn Berechnung kürzer gedauert hat, als erwartet
			// wird die Sequenz erst entsprechend später gesendet,
			// damit sie zum Zustand passt
				if (time2 < delta)
					time1 = delta;
				else
					time1 = time2;
				//time1 = delta; //MOD, LÖSCHEN

			// echtes System integrieren
				
				ph_real.n_dis = (int)(1000.0*time1) + 1;
				for (state = 0; state < ph_real.n_ode; state++)
					ph_real.X[ph_real.x_index(0, state)] = start[state];
				thrsh = 0; tmp = 1 / (ph_optim.T[1] - ph_optim.T[0]);
				tmp2 = ph_optim.u(0, 0)*ph_optim.T[1] - ph_optim.u(1, 0)*ph_optim.T[0];
				ph_real.X[ph_real.u_index(1, 0)] = (ph_optim.u(1, 0) - ph_optim.u(0, 0))*ph_real.T[1] + tmp2;
				ph_real.X[ph_real.u_index(1, 0)] *= tmp;
				for (i = 0; i < ph_real.n_dis; i++) {
					steps = ph_real.Integrate(twparameter.butchertableau);
					if (ph_real.T[i] > ph_optim.T[thrsh + 1]) {
						thrsh++;
						tmp = 1 / (ph_optim.T[thrsh + 1] - ph_optim.T[thrsh]);
						tmp2 = ph_optim.u(thrsh, 0)*ph_optim.T[thrsh + 1] - ph_optim.u(thrsh + 1, 0)*ph_optim.T[thrsh];
					}
					for (state = 0; state < ph_real.n_ode; state++)
						ph_real.X[ph_real.x_index(0, state)] = ph_real.x(1,state);
					ph_real.X[ph_real.u_index(0, 0)] = ph_real.X[ph_real.u_index(1, 0)];
					ph_real.X[ph_real.u_index(1, 0)] = (ph_optim.u(thrsh + 1, 0) - ph_optim.u(thrsh, 0))*ph_real.T[i] + tmp2;
					ph_real.X[ph_real.u_index(1, 0)] *= tmp;
				}

			// Textausgabe
/*
				for (i = 0; i <= thrsh; i++) {
					mpcfile2 << time + ph_real.T[i] << ", " << ph_real.u(i, 0) << ", ";
					for (state = 0; state < ph_real.n_ode; state++) {
						mpcfile2 << ph_real.x(i, state) << ", ";
					}
					mpcfile2 << endl;
				}
*/

			// Änderungen rückgängig machen
				ph_real.n_dis = 2;
				
			// Messsignal nach Startschätzung + Optimierung als neuen Startwert
				for (state = 0; state < ph_real.n_ode; state++)
					start[state] = ph_real.x(thrsh, state);
				start[4] = 0; //Energie wieder bei 0 starten
				

			// mittlere erwartete Zeit für Startschätzung + Optimierung
			delta = (delta + time2) / 2;
			// Gesamtzeit weiter stellen
			time += time1;
		}
		mpcfile.close();
		mpcfile2.close();
		mpcfile3.close();
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

/*
LARGE_INTEGER time[2];
LARGE_INTEGER looptime[15];
LARGE_INTEGER deltalt[15];
LARGE_INTEGER ltmean;
LARGE_INTEGER ltsigma;
LARGE_INTEGER Frequency;

QueryPerformanceFrequency(&Frequency);
QueryPerformanceCounter(&time[0]);
QueryPerformanceCounter(&time[1]);
for (i = 0; i < 15; i++) {
looptime[i].QuadPart = 0;
deltalt[i].QuadPart = 0;
}
ltmean.QuadPart = 0;
ltsigma.QuadPart = 0;
*/
/*
QueryPerformanceCounter(&time[1]);
int i = K % 15;
ltsigma.QuadPart -= deltalt[i].QuadPart;
ltmean.QuadPart -= looptime[i].QuadPart;
looptime[i].QuadPart = time[1].QuadPart - time[0].QuadPart;
ltmean.QuadPart += looptime[i].QuadPart;
deltalt[i].QuadPart = abs(looptime[i].QuadPart - ltmean.QuadPart / 15);
ltsigma.QuadPart += deltalt[i].QuadPart;
QueryPerformanceCounter(&time[0]);
printf("mittl. Loopzeit: %d ms, ", ltmean.QuadPart * 1000/15 / Frequency.QuadPart);
printf("Abweichung: %d ms\n", ltsigma.QuadPart * 1000/15 / Frequency.QuadPart);
*/
/*
for (int i = 0; i < 10; i++){
printf("Loopzeit: %d, ", looptime[i].QuadPart);
printf("Abweichung: %d\n", deltalt[i].QuadPart);
printf("Loopzeit: %d ms, ", looptime[i].QuadPart * 1000 / Frequency.QuadPart);
printf("Abweichung: %d ms\n", deltalt[i].QuadPart * 100 / Frequency.QuadPart);
}
//*/
