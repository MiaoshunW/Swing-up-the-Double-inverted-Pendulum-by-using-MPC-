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
const double ziel[] = { 0, 0, 0, 0, 0 };

double weight[] = { 1.0, 0.0, 1.0, 0.0, 0.02, 1.0 };
// Wagenlage, Wagengeschw., Pendellage, Pendelgeschw., Stellenergie, Pendelenergie
// werden jeweils auf Grenze, bzw. Pi, normiert 
const double up_bnd[] = { 1.2, 0.38, 1.2, 1e20, 25, 0.5 };
const double lw_bnd[] = { -1.2, -0.38, -1.2, -1e20, -25, 0 };
// Sollgeschw., Wagenlage, Wagengeschw., Pendellage, Pendelgeschw., Stellenergie

const double dis_times[] = { 0.0, 0.06, 0.19, 0.36, 0.58, 0.7 };// { 0.0, 0.04, 0.12, 0.3, 0.4, 0.5 };

// Wenn die aktuelle Berechnungszeit um mehr als pre_calc_time geringer als
// die geschätzte Berechnungszeit ausfällt, wird die nächste Startschätzung nicht
// erst bei der Übergabezeit der Steuerfolge gestartet, sondern direkt
const double pre_calc_time = 0.008;

// Ursprüngliche Annahme der Berechnungsdauer
const double ext_estim_time = 0.04;

double control[6];
double states[6][4];

int error;

double gl,l,J_eff;
double max_bnd[5];

const double  a[2] = { 81.48, 5135.0 };
const double  c = (6 / ((4 * 0.5) + ((0.02*0.02) / 0.5)));
const double  g = 9.81;
const double  T1 = 0.015;

class MPC_Pendel_optim : public TransWorhp {

public:

	MPC_Pendel_optim(int dis) : TransWorhp("MPC_Pendel_optim", dis, 5, 1, 0, 0, 0) {}

	void GetXTitle(int d, char *s) {
		if (d == 0) strcpy(s, "position x of the cart");
		if (d == 1) strcpy(s, "cart velocity");
		if (d == 2) strcpy(s, "pendulum angle ");
		if (d == 3) strcpy(s, "pendulum angular velocity ");
		if (d == 4) strcpy(s, "input energy");
	}

	void GetUTitle(int d, char *s) {
		if (d == 0) strcpy(s, "input velocity");
	}


	//--------- Startschätzung der Optimierung (optional) ---------------------//

	void x_init(double *x, int i, int dis) {
		for (j = 0; j < n_ode; j++) {
			//x[j] = (start[j] * (dis - i) + ziel[j] * i) / dis;
			x[j] = states[i][j];
		}
	}
	void u_init(double *u, int i, int dis) {
		//u[0] = 0.0;
		u[0] = control[i];
	}


	//--------- Objective Function -------------------------------------------//

	double obj() {
		double tmp, alpha, ret = weight[2] * Pi * Pi;
		j = n_dis - 1;

		alpha = fabs(fmod(x(j, 2) - ziel[2] + Pi, Pi2) - Pi);

		tmp = (x(j, 1) - ziel[1]);
		ret += tmp*tmp * weight[1];

		tmp = (x(j, 3) - ziel[3]);
		ret += tmp*tmp * weight[3];

		ret += x(j, 4) * weight[4];

		tmp = cos(x(j, 2)) - cos(ziel[2]);
		tmp *= x(j, 1)*x(j, 3)*l + gl;
		tmp += x(j, 3)*x(j, 3)*J_eff;
		tmp += x(j, 1)*x(j, 1);
		ret += tmp*tmp* weight[5];

		ret *= alpha / Pi;

		tmp = (x(j, 0) - ziel[0]);
		ret += tmp*tmp * weight[0];

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
		dx[0] = x[1];
		dx[1] = (u[0] - x[1]) / T1;
		dx[2] = x[3];
		dx[3] = -1 * (((u[0] - x[1]) / T1)*c*cos(x[2])) + (c*g*sin(x[2])) - (0.01*c*x[3]);
		dx[4] = (u[0] * u[0]);
	}


	//--------- Jacobian for Derivatives (Optional) -------------------------//

	bool ode_structure(DiffStructure &s) {
		//return false;
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
		//return false;

		//dx[0] = dx[0] = x[1];
		s(0, x_indexode(0)) = 0;
		s(0, x_indexode(1)) = 1;
		s(0, x_indexode(2)) = 0;
		s(0, x_indexode(3)) = 0;
		s(0, x_indexode(4)) = 0;
		s(0, u_indexode(0)) = 0;

		// dx[1] = (u[0] - x[1]) / T1;
		s(1, x_indexode(0)) = 0;
		s(1, x_indexode(1)) = -1 / T1;
		s(1, x_indexode(2)) = 0;
		s(1, x_indexode(3)) = 0;
		s(1, x_indexode(4)) = 0;
		s(1, u_indexode(0)) = 1 / T1;

		// dx[2] = x[3];
		s(2, x_indexode(0)) = 0;
		s(2, x_indexode(1)) = 0;
		s(2, x_indexode(2)) = 0;
		s(2, x_indexode(3)) = 1;
		s(2, x_indexode(4)) = 0;
		s(2, u_indexode(0)) = 0;

		// dx[3] = (((u[0] - x[1]) / T1)*c*cos(x[2])) + (c*g*sin(x[2])) - (0.01*c*x[3]);
		s(3, x_indexode(0)) = 0;
		s(3, x_indexode(1)) = -c/T1*cos(x[2]);
		s(3, x_indexode(2)) = c*(g*cos(x[2])-x[1]/T1*sin(x[2]));
		s(3, x_indexode(3)) = -0.01*c;
		s(3, x_indexode(4)) = 0;
		s(3, u_indexode(0)) = c/T1*cos(x[2]);

		// dx[4] = (u[0] * u[0]);
		s(4, x_indexode(0)) = 0;
		s(4, x_indexode(1)) = 0;
		s(4, x_indexode(2)) = 0;
		s(4, x_indexode(3)) = 0;
		s(4, x_indexode(4)) = 0;
		s(4, u_indexode(0)) = 2*u[0];

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

	MPC_Pendel_real(int dis) : TransWorhp("MPC_Pendel_real", dis, 6, 1, 0, 0, 0) {}

	void GetXTitle(int d, char *s) {
		if (d == 0) strcpy(s, "position x of the cart");
		if (d == 1) strcpy(s, "cart velocity");
		if (d == 2) strcpy(s, "cart acceleration");
		if (d == 3) strcpy(s, "pendulum angle ");
		if (d == 4) strcpy(s, "pendulum angular velocity ");
		if (d == 5) strcpy(s, "input energy");
	}

	void GetUTitle(int d, char *s) {
		if (d == 0) strcpy(s, "input velocity");
	}


	//--------- Startschätzung der Optimierung (optional) ---------------------//
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

	void ode(double *dx, double t, const double *x, const double *u, const double *p) {
		//a[2] = { 81.48, 5135.0 };
		dx[0] = x[1];
		dx[1] = x[2];
		dx[2] = a[1] * (u[0] - x[1]) - a[0] * x[2];
		dx[3] = x[4];
		dx[4] = (-c * x[2] * cos(x[3])) + (c*g*sin(x[3])) - (0.01*c*x[4]);
		dx[5] = (u[0] * u[0]);
	}


	//--------- Jacobian for Derivatives (Optional) -------------------------//

	bool ode_structure(DiffStructure &s) {
		//return false;

		s(0, x_indexode(1));

		s(1, x_indexode(2));

		s(2, x_indexode(1));
		s(2, x_indexode(2));
		s(2, u_indexode(0));

		s(3, x_indexode(4));

		s(4, x_indexode(2));
		s(4, x_indexode(3));
		s(4, x_indexode(4));

		s(5, u_indexode(0));

		return true;
	}

	bool ode_diff(DiffStructure& s, double t, const double* x, const double* u, const double* p) {
		//return false;

		//dx[0] = x[1];
		s(0, x_indexode(0)) = 0;
		s(0, x_indexode(1)) = 1;
		s(0, x_indexode(2)) = 0;
		s(0, x_indexode(3)) = 0;
		s(0, x_indexode(4)) = 0;
		s(0, x_indexode(5)) = 0;
		s(0, u_indexode(0)) = 0;

		// dx[1] = a[1] * x[2];
		s(1, x_indexode(0)) = 0;
		s(1, x_indexode(1)) = 0;
		s(1, x_indexode(2)) = a[1];
		s(1, x_indexode(3)) = 0;
		s(1, x_indexode(4)) = 0;
		s(1, x_indexode(5)) = 0;
		s(1, u_indexode(0)) = 0;

		// dx[2] = a[1] * (u[0] - x[1]) - a[0] * x[2];
		s(2, x_indexode(0)) = 0;
		s(2, x_indexode(1)) = -a[1];
		s(2, x_indexode(2)) = -a[0];
		s(2, x_indexode(3)) = 0;
		s(2, x_indexode(4)) = 0;
		s(2, x_indexode(5)) = 0;
		s(2, u_indexode(0)) = a[1];

		// dx[3] = x[4];
		s(3, x_indexode(0)) = 0;
		s(3, x_indexode(1)) = 0;
		s(3, x_indexode(2)) = 0;
		s(3, x_indexode(3)) = 0;
		s(3, x_indexode(4)) = 1;
		s(3, x_indexode(5)) = 0;
		s(3, u_indexode(0)) = 0;

		// dx[4] = (-c * x[2] * cos(x[3])) + (c*g*sin(x[3])) - (0.01*c*x[4]);
		s(4, x_indexode(0)) = 0;
		s(4, x_indexode(1)) = 0;
		s(4, x_indexode(2)) = -c * cos(x[3]);
		s(4, x_indexode(3)) = c * g * cos(x[3]) + c * x[2] * sin(x[3]);
		s(4, x_indexode(4)) = -0.01*c;
		s(4, x_indexode(5)) = 0;
		s(4, u_indexode(0)) = 0;

		// dx[4] = (u[0] * u[0]);
		s(5, x_indexode(0)) = 0;
		s(5, x_indexode(1)) = 0;
		s(5, x_indexode(2)) = 0;
		s(5, x_indexode(3)) = 0;
		s(5, x_indexode(4)) = 0;
		s(5, x_indexode(5)) = 0;
		s(5, u_indexode(0)) = 2*u[0];

		return true;
	}

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


/////////////////////////////////////////////////////////////////////////////

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

	twparameter.NDIS = 6;
	TWfolder folder_simu(&twparameter, 0);
	twparameter.NDIS = 6;
	TWfolder folder_optim(&twparameter, 0);
	twparameter.NDIS = 6;
	TWfolder folder_real(&twparameter, 0);

	MPC_Pendel_optim ph_optim(6);
	MPC_Pendel_optim ph_simu(6);
	MPC_Pendel_real ph_real(6);

	folder_optim.Add(&ph_optim);
	folder_real.Add(&ph_real);
	folder_simu.Add(&ph_simu);

	ofstream mpcfile("mpc.dat");
	ofstream mpcfile2("mpc2.dat");
	ofstream mpcfile3("mpc3.dat");
	ofstream mpcfile4("mpc4.dat");

	int state, i, steps, thrsh, thrsh2;
	
	double base_time_optim, base_time_real, start_time, calc_time_last;
	double calc_time_current, calc_time_estim, sync_time_current, sync_time_estim;
	double temp_u, compare_time, temp_time[3];

	try
	{
		for (i = 0; i < ph_optim.n_dis; i++)
			ph_optim.T[i] = dis_times[i];
		for (i = 0; i < ph_real.n_dis; i++)
			ph_real.T[i] = dis_times[i];
		for (i = 0; i < ph_simu.n_dis; i++)
			ph_simu.T[i] = dis_times[i];

		//*
		for (i = 0; i < ph_optim.n_dis; i++) {
		control[i] = 0.0;
		for (state = 0; state < ph_optim.n_ode; state++)
		states[i][state] = (start[state] * (ph_optim.n_dis - i) + ziel[state] * i) / ph_optim.n_dis;
		}
		//*/

		folder_optim.Init();
		folder_optim.Loop(1, 0);
		ph_optim.n_dis = 5;

		calc_time_estim = ext_estim_time;
		base_time_optim = 0.0;
		base_time_real = 0.0;
		calc_time_current = 0.0;

		folder_real.Init();
		folder_simu.Init();
		ph_real.X[ph_real.x_index(0, 0)] = 0.0;
		ph_real.X[ph_real.x_index(0, 1)] = 0.0;
		ph_real.X[ph_real.x_index(0, 2)] = 0.0;
		ph_real.X[ph_real.x_index(0, 3)] = Pi;
		ph_real.X[ph_real.x_index(0, 4)] = 0.0;
		ph_real.X[ph_real.x_index(0, 5)] = 0.0;

		for (int K = 0; K < 60; K++) { //INFINITY

			start_time = 0.001*(double)GetTickCount();

			// Startschätzung aus Anfangszustand und Steuerung integrieren
				
				//Restberechnungszeit bestimmen
				sync_time_estim = base_time_optim - base_time_real + calc_time_estim;
				//ggf. kürzer, aber mindestens einen Schritt weit
				sync_time_estim = max(sync_time_estim, dis_times[1]);

				// Anfangszustand festlegen
				for (state = 0; state < ph_simu.n_ode; state++)
					ph_simu.X[ph_simu.x_index(0, state)] = start[state];

				// Steuerung festlegen
				thrsh = 1;
				while (dis_times[thrsh] < sync_time_estim) thrsh++;
				if (thrsh >= ph_simu.n_dis) {
					mpcfile << sync_time_estim << "s: last optim run took too long!\n";
					exit(2);
				}
				for (i = 0; i < thrsh; i++)
					ph_simu.X[ph_simu.u_index(i, 0)] = ph_optim.u(i, 0);
				//lineare Interpolation
				ph_simu.X[ph_simu.u_index(thrsh, 0)] = (ph_optim.u(thrsh, 0)*(sync_time_estim - ph_optim.T[thrsh - 1])
					+ ph_optim.u(thrsh - 1, 0)*(ph_optim.T[thrsh] - sync_time_estim)) / (ph_optim.T[thrsh] - ph_optim.T[thrsh - 1]);
				ph_simu.T[thrsh] = sync_time_estim;
				ph_simu.n_dis = thrsh + 1;

				// Systemmodell integrieren
				steps = ph_simu.Integrate(twparameter.butchertableau);

				// Änderung rückgängig machen
				ph_simu.n_dis = 6;

			// Zeit für Startschätzung
			calc_time_current += 0.001*(double)GetTickCount() - start_time;

			// Steuerung echtes System festlegen
			for (i = 0; i < ph_real.n_dis; i++)
				ph_real.X[ph_real.u_index(i, 0)] = ph_optim.u(i, 0);


			// Textausgabe
				mpcfile << "Unabänderliche Steuersignale: " << endl;
				for (i = 0; i < thrsh; i++)
					mpcfile << base_time_optim + ph_simu.T[i] << " s: " << ph_simu.u(i, 0) << endl;
				mpcfile << endl << "Erwartete Zustände: ";
				for (i = 0; i <= thrsh; i++) {
					mpcfile << endl << base_time_optim + ph_simu.T[i] << " s: ";
					mpcfile3 << base_time_optim + ph_simu.T[i] << ", " << ph_simu.u(i, 0);
					for (state = 0; state < ph_simu.n_ode; state++) {
						mpcfile << ph_simu.x(i, state) << ", ";
						mpcfile3 << ", " << ph_simu.x(i, state);
					}
					if ((i == 0) && (base_time_real==base_time_optim))
						mpcfile3 << ", 1";
					else
						mpcfile3 << ", 0";
					mpcfile3 << endl;
				}
				mpcfile << endl << endl;

			start_time = 0.001*(double)GetTickCount();

			// Optimierung initialisieren mit Startschätzung
				for (state = 0; state < ph_optim.n_ode; state++)
					start[state] = ph_simu.x(thrsh, state);

			// Optimierung
				//*
				// letzten Durchlauf als Startschätzung für Optimierung übernehmen
				control[0] = ph_simu.u(thrsh, 0);
				compare_time = sync_time_estim + dis_times[1];
				for (i = 1; compare_time < dis_times[ph_optim.n_dis-1]; i++) {
				while (dis_times[thrsh] < compare_time) thrsh++;
				temp_time[0] = compare_time - ph_optim.T[thrsh - 1];
				temp_time[1] = ph_optim.T[thrsh] - compare_time;
				temp_time[2] = ph_optim.T[thrsh] - ph_optim.T[thrsh - 1];
				control[i] = (ph_optim.u(thrsh, 0)*temp_time[0] + ph_optim.u(thrsh - 1, 0)*temp_time[1])/temp_time[2];
				for (state = 0; state < ph_optim.n_ode; state++) {
				states[i][state] = (ph_optim.x(thrsh, state)*temp_time[0] + ph_optim.x(thrsh - 1, state)*temp_time[1]) / temp_time[2];
				}
				compare_time = sync_time_estim + dis_times[i+1];
				}
				//*/
				// oben weniger Zeitpunkte berücksichtigen
				ph_optim.n_dis = 3 + (int)fabs(fmod(start[2] + Pi, Pi2) - Pi);
				ph_optim.n_dis = (ph_optim.n_dis > 3) ? ph_optim.n_dis : 4;
				ph_optim.n_dis = (ph_optim.n_dis < 6) ? ph_optim.n_dis : 5;

				folder_optim.Reinit();
				folder_optim.Loop(1, 0);

			// Zeit für Startschätzung + Optimierung
			calc_time_current += 0.001*(double)GetTickCount() - start_time;

			// Textausgabe
				mpcfile << "Optimale Steuerung: " << endl;
				for (i = 0; i < ph_optim.n_dis; i++)
					mpcfile << base_time_optim + calc_time_estim + ph_optim.T[i] << " s: " << ph_optim.u(i, 0) << endl;
				mpcfile << endl << "Angestrebte Zustände: ";
				for (i = 0; i < ph_optim.n_dis; i++) {
					mpcfile << endl << base_time_optim + calc_time_estim + ph_optim.T[i] << " s: ";
					mpcfile3 << base_time_optim + calc_time_estim + ph_optim.T[i] << ", " << ph_optim.u(i, 0);
					for (state = 0; state < ph_optim.n_ode; state++) {
						mpcfile << ph_optim.x(i, state) << ", ";
						mpcfile3 << ", " << ph_optim.x(i, state);
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

			//Zeiten synchronisieren
			sync_time_current = calc_time_current + base_time_optim - base_time_real;
			sync_time_estim = calc_time_estim + base_time_optim - base_time_real;

			// Wenn Berechnung kürzer gedauert hat, als erwartet
			// wird die Sequenz erst entsprechend später gesendet,
			// damit sie zum Zustand passt. Die nächste
			// Berechnung kann trotzdem schon beginnen, wenn der Abstand
			// den Aufwand rechtfertigt

			if (sync_time_current < sync_time_estim - pre_calc_time)
			{
			// echtes System integrieren

				mpcfile4 << sync_time_estim - sync_time_current << endl;
				// Steuerung festlegen
				thrsh = 1;
				while (dis_times[thrsh] < sync_time_current) thrsh++;
				//lineare Interpolation
				temp_u = (ph_real.u(thrsh, 0)*(sync_time_current - ph_real.T[thrsh - 1])
					+ ph_real.u(thrsh - 1, 0)*(ph_real.T[thrsh] - sync_time_current)) / (ph_real.T[thrsh] - ph_real.T[thrsh - 1]);
				thrsh2 = thrsh;
				while (dis_times[thrsh2] < sync_time_estim) thrsh2++;
				thrsh++;
				//lineare Interpolation
				ph_real.X[ph_real.u_index(thrsh, 0)] = (ph_real.u(thrsh2, 0)*(sync_time_estim - ph_real.T[thrsh2 - 1])
					+ ph_real.u(thrsh2 - 1, 0)*(ph_real.T[thrsh2] - sync_time_estim)) / (ph_real.T[thrsh2] - ph_real.T[thrsh2 - 1]);
				ph_real.X[ph_real.u_index(thrsh-1, 0)] = temp_u;
				ph_real.T[thrsh] = sync_time_estim;
				ph_real.T[thrsh - 1] = sync_time_current;
				ph_real.n_dis = thrsh + 1;

				// System integrieren
				steps = ph_real.Integrate(twparameter.butchertableau);

			// Textausgabe
				mpcfile << "Ausgeführte Steuersignale: " << endl;
				for (i = 0; i < thrsh; i++) {
					mpcfile << base_time_real + ph_real.T[i] << " s: " << ph_real.u(i, 0) << endl;
				}
				mpcfile << endl << "Echte Zustände: ";
				for (i = 0; i <= thrsh; i++) {
					mpcfile << endl << base_time_real + ph_real.T[i] << " s: ";
					mpcfile2 << base_time_real + ph_real.T[i] << ", " << ph_real.u(i, 0) << ", ";
					for (state = 0; state < ph_real.n_ode; state++) {
						mpcfile << ph_real.x(i, state) << ", ";
						mpcfile2 << ph_real.x(i, state) << ", ";
					}
					mpcfile2 << endl;
				}
				mpcfile << endl << endl;

			// Änderungen rückgängig machen
				ph_real.T[thrsh - 1] = dis_times[thrsh - 1];
				ph_real.T[thrsh] = dis_times[thrsh];
				ph_real.n_dis = 6;

			// reales System am Übergabepunkt reinitialisieren
			// und Anfangszustand Startschätzung festlegen
				for (state = 0; state < ph_real.n_ode-1; state++) {
					ph_real.X[ph_real.x_index(0, state)] = ph_real.x(thrsh, state);
					ph_simu.X[ph_simu.x_index(0, state)] = ph_real.x(thrsh - 1, state);
				}
				ph_real.X[ph_real.x_index(0, ph_real.n_ode-1)] = 0.0; //Energie wieder bei 0 starten
				ph_simu.X[ph_simu.x_index(0, ph_real.n_ode-1)] = 0.0;

			calc_time_last = calc_time_current; //Zwischenspeichern

			// Startschätzung aus Anfangszustand und Steuerung integrieren
			start_time = 0.001*(double)GetTickCount();

				// Steuerung festlegen
				thrsh = 1;
				while (dis_times[thrsh] < sync_time_current) thrsh++;
				//lineare Interpolation für aktuellen Punkt
				ph_simu.X[ph_simu.u_index(0, 0)] = (ph_real.u(thrsh, 0)*(sync_time_current - ph_real.T[thrsh - 1])
					+ ph_real.u(thrsh - 1, 0)*(ph_real.T[thrsh] - sync_time_current)) / (ph_real.T[thrsh] - ph_real.T[thrsh - 1]);
				i = 1;
				thrsh2 = thrsh;
				while (dis_times[thrsh2] < sync_time_estim) {
					ph_simu.X[ph_simu.u_index(i, 0)] = ph_real.u(thrsh2,0);
					ph_simu.T[i] = dis_times[thrsh2] - sync_time_current;
					thrsh2++; i++;
				}
				thrsh = i;
				//lineare Interpolation für Übergabepunkt
				ph_simu.X[ph_real.u_index(thrsh, 0)] = (ph_real.u(thrsh2, 0)*(sync_time_estim - ph_real.T[thrsh2 - 1])
					+ ph_real.u(thrsh2 - 1, 0)*(ph_real.T[thrsh2] - sync_time_estim)) / (ph_real.T[thrsh2] - ph_real.T[thrsh2 - 1]);
				ph_simu.T[thrsh] = sync_time_estim - sync_time_current;
				ph_simu.n_dis = thrsh + 1;

				// System integrieren
				steps = ph_simu.Integrate(twparameter.butchertableau);


				// Textausgabe
				mpcfile << "Unabänderliche Steuersignale: " << endl;
				for (i = 0; i < thrsh; i++)
					mpcfile << base_time_optim + ph_simu.T[i] << " s: " << ph_simu.u(i, 0) << endl;
				mpcfile << endl << "Erwartete Zustände: ";
				for (i = 0; i <= thrsh; i++) {
					mpcfile << endl << base_time_optim + ph_simu.T[i] << " s: ";
					mpcfile3 << base_time_optim + ph_simu.T[i] << ", " << ph_simu.u(i, 0);
					for (state = 0; state < ph_simu.n_ode; state++) {
						mpcfile << ph_simu.x(i, state) << ", ";
						mpcfile3 << ", " << ph_simu.x(i, state);
					}
					if (i == 0)
						mpcfile3 << ", 1";
					else
						mpcfile3 << ", 0";
					mpcfile3 << endl;
				}
				mpcfile << endl << endl;

				// Integral als neuen Startwert
				for (state = 0; state < ph_simu.n_ode; state++)
					start[state] = ph_simu.x(thrsh, state);
				start[4] = 0; //Energie wieder bei 0 starten

				// Änderungen rückgängig machen
				for (i = 0; i < ph_simu.n_dis; i++)
					ph_simu.T[i] = dis_times[i];
				ph_simu.n_dis = 6;

			calc_time_current = 0.001*(double)GetTickCount() - start_time;

			// Zeitstempel weiter stellen
				base_time_real += sync_time_estim;
				base_time_optim += calc_time_last;

			// mittlere erwartete Zeit für Startschätzung + Optimierung
				calc_time_estim = (calc_time_estim + calc_time_last) / 2;
			}
			else
			// bei weniger als pre_calc_time Restlaufzeit, genau getroffener Schätzung oder
			// zu langer Berechnung Steuerfolge
			{
			// echtes System integrieren

				//Übergabezeitpunkt
				sync_time_current = max(sync_time_current, sync_time_estim);

				// Steuerung festlegen
				thrsh = 1;
				while (dis_times[thrsh] < sync_time_current) thrsh++;
				//lineare Interpolation
				ph_real.X[ph_real.u_index(thrsh, 0)] = (ph_real.u(thrsh, 0)*(sync_time_current - ph_real.T[thrsh - 1])
					+ ph_real.u(thrsh - 1, 0)*(ph_real.T[thrsh] - sync_time_current)) / (ph_real.T[thrsh] - ph_real.T[thrsh - 1]);
				ph_real.T[thrsh] = sync_time_current;
				ph_real.n_dis = thrsh + 1;

				// System integrieren
				steps = ph_real.Integrate(twparameter.butchertableau);

			// Textausgabe
				mpcfile << "Ausgeführte Steuersignale: " << endl;
				for (i = 0; i < thrsh; i++) {
					mpcfile << base_time_real + ph_real.T[i] << " s: " << ph_real.u(i, 0) << endl;
				}
				mpcfile << endl << "Echte Zustände: ";
				for (i = 0; i <= thrsh; i++) {
					mpcfile << endl << base_time_real + ph_real.T[i] << " s: ";
					mpcfile2 << base_time_real + ph_real.T[i] << ", " << ph_real.u(i, 0) << ", ";
					for (state = 0; state < ph_real.n_ode; state++) {
						mpcfile << ph_real.x(i, state) << ", ";
						mpcfile2 << ph_real.x(i, state) << ", ";
					}
					mpcfile2 << endl;
				}
				mpcfile << endl << endl;

			// Änderungen rückgängig machen
				ph_real.T[thrsh] = dis_times[thrsh];
				ph_real.n_dis = 6;

			// Messsignal nach Startschätzung + Optimierung als neuen Startwert,
			// reales System am Übergabepunkt reinitialisieren
				start[0] = ph_real.x(thrsh, 0);
				ph_real.X[ph_real.x_index(0, 0)] = start[0];
				start[1] = ph_real.x(thrsh, 1);
				ph_real.X[ph_real.x_index(0, 1)] = start[1];
				ph_real.X[ph_real.x_index(0, 2)] = ph_real.x(thrsh, 2);
				start[2] = ph_real.x(thrsh, 3);
				ph_real.X[ph_real.x_index(0, 3)] = start[2];
				start[3] = ph_real.x(thrsh, 4);
				ph_real.X[ph_real.x_index(0, 4)] = start[3];
				start[4] = 0.0; //Energie wieder bei 0 starten
				ph_real.X[ph_real.x_index(0, 5)] = 0.0;

			// Gesamtzeit weiter stellen
				base_time_real += sync_time_current;
				base_time_optim = base_time_real;

			// mittlere erwartete Zeit für Startschätzung + Optimierung
				calc_time_estim = (calc_time_estim + calc_time_current) / 2;

				calc_time_current = 0.0;
			}
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
