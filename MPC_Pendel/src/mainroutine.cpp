#define SDL_MAIN_HANDLED

#ifdef WIN32
#include "windows.h"
#endif

#include <conio.h>
#include <fstream>
using namespace std;

#include "subroutine.h"
#include "quickselect.h"

int main(void)
{
	//double base_times[6] = { 0.0, 0.035, 0.19, 0.36, 0.63, 0.7 };
	//double base_times[6] = { 0.0, 0.06, 0.19, 0.31, 0.63, 0.7 };
	//double base_times[6] = { 0.0, 0.06, 0.19, 0.385, 0.555, 0.625 };
	//double base_times[6] = { 0.0, 0.06, 0.19, 0.355, 0.525, 0.625 };
	//double base_times[6] = { 0.0, 0.1, 0.2, 0.3, 0.4, 0.6 };
	double base_times[6] = { 0.0, 0.05, 0.2, 0.3, 0.45, 0.55 };

	double Schrittweite[5] = { 0.005, 0.005, 0.005, 0.005, 0.005 };
	
	const int Anzahl[5] = { 7,7,7,7,7 }; // copy below
	double obj_res[5][5][5][5][5];
	int bal_res[5][5][5][5][5];

	const int runs = 5; // copy below
	double result[5];

	double dis_times[6], med_result, mean_time, time1;
	int state, balanced, c[5], c_start[5];
	int *p_bal_res;
	/*
	ofstream mpcfile("optim.dat");
	ofstream mpcfile2("optim_obj.dat");
	ofstream mpcfile3("optim_bal.dat");
	mpcfile2.close();
	mpcfile3.close();
	ofstream mpcfile4("optim_log.dat");
	mpcfile4.close();

	c_start[0] = 0;
	c_start[1] = 0;
	c_start[2] = 0;
	c_start[3] = 0;
	c_start[4] = 0;
	c_start[0] = 0;
	c_start[1] = 1;
	c_start[2] = 6;
	c_start[3] = 1;
	c_start[4] = 6;
	c_start[0] = 0;
	c_start[1] = 2;
	c_start[2] = 2;
	c_start[3] = 4;
	c_start[4] = 3;
	c_start[0] = 0;
	c_start[1] = 2;
	c_start[2] = 5;
	c_start[3] = 1;
	c_start[4] = 0;
	c_start[0] = 0;
	c_start[1] = 2;
	c_start[2] = 5;
	c_start[3] = 1;
	c_start[4] = 3;
	//*/
	//*
	ofstream mpcfile("optim.dat", ofstream::app);
	ofstream mpcfile2("optim_obj.dat", ofstream::app);
	ofstream mpcfile3("optim_bal.dat", ofstream::app);
	mpcfile2.close();
	mpcfile3.close();
	ofstream mpcfile4("optim_log.dat", ofstream::app);
	mpcfile4.close();

	c_start[0] = 1;
	c_start[1] = 5;
	c_start[2] = 6;
	c_start[3] = 5;
	c_start[4] = 6;
	//*/

	for (c[0] = c_start[0]; c[0] < Anzahl[0]; c[0]++) {
		if (c[0] > c_start[0])
			c_start[1] = 0;
		for (c[1] = c_start[1]; c[1] < Anzahl[1]; c[1]++) {
			if (c[1] > c_start[1])
				c_start[2] = 0;
			for (c[2] = c_start[2]; c[2] < Anzahl[2]; c[2]++) {
				if (c[2] > c_start[2])
					c_start[3] = 0;
				for (c[3] = c_start[3]; c[3] < Anzahl[3]; c[3]++) {
					if (c[3] > c_start[3])
						c_start[4] = 0;
					for (c[4] = c_start[4]; c[4] < Anzahl[4]; c[4]++) {

						dis_times[0] = base_times[0];
						dis_times[1] = base_times[1] + Schrittweite[0] * (double)(c[0] - (Anzahl[0] - 1) / 2);
						dis_times[2] = base_times[2] + Schrittweite[1] * (double)(c[1] - (Anzahl[1] - 1) / 2);
						dis_times[3] = base_times[3] + Schrittweite[2] * (double)(c[2] - (Anzahl[2] - 1) / 2);
						dis_times[4] = base_times[4] + Schrittweite[3] * (double)(c[3] - (Anzahl[3] - 1) / 2);
						dis_times[5] = base_times[5] + Schrittweite[4] * (double)(c[4] - (Anzahl[4] - 1) / 2);

						if ((dis_times[0] >= dis_times[1]) || (dis_times[1] >= dis_times[2]) || (dis_times[2] >= dis_times[3]) || (dis_times[3] >= dis_times[4]) || (dis_times[4] >= dis_times[5])) {
							ofstream mpcfile2("optim_obj.dat", ofstream::app);
							ofstream mpcfile3("optim_bal.dat", ofstream::app);
							mpcfile2 << 1e20 << ", ";
							mpcfile3 << 0 << ", ";
							mpcfile2.close();
							mpcfile3.close();
							continue;
						}

						mpcfile << "0.000, " << dis_times[1] << ", " << dis_times[2] << ", ";
						mpcfile << dis_times[3] << ", " << dis_times[4] << ", " << dis_times[5] << "\n";
						mpcfile.close();

						mean_time = 0;
						p_bal_res = &(bal_res[c[0]][c[1]][c[2]][c[3]][c[4]]);
						*p_bal_res = 0;
						try {
							for (int i = 0; i < runs; i++) {
								time1 = -0.001*(double)GetTickCount();
								result[i] = subroutine(dis_times, &state, &balanced, i);

								ofstream mpcfile4("optim_log.dat", ofstream::app);
								mpcfile4 << result[i] << "," << state << "; ";
								mpcfile4.close();

								*p_bal_res += balanced;
								mean_time += time1 + 0.001*(double)GetTickCount();
								if (state == 2) {
									dis_times[5] += 0.1;
									i--;
								}
								else {
									if (state == 1) {
										mpcfile << state << ", " << result;
										mpcfile.close();
										exit(state);
									}
									if (state == 3) { // Wagenbegrenzung verletzt
										result[i] *= 100;
										state = 0;
									}
									if ((state == 4) || (state == 2)) // Zeitbeschränkung verletzt
										break;
								}
								if (result[i] == 0) {
									result[i] = 1e20;
									mpcfile << "0.000, " << dis_times[1] << ", " << dis_times[2] << ", ";
									mpcfile << dis_times[3] << ", " << dis_times[4] << ", " << dis_times[5] << "\n";
									break;
								}

							}
							ofstream mpcfile4("optim_log.dat", ofstream::app);
							mpcfile4 << endl;
							mpcfile4.close();
							mean_time /= (double)runs;
							med_result = quick_select(result, runs);
							if ((state == 4) || (state == 2)) { // Zeitbeschränkung verletzt
								med_result = 1e20;
							}
							obj_res[c[0]][c[1]][c[2]][c[3]][c[4]] = med_result;

							ofstream mpcfile("optim.dat", ofstream::app);
							ofstream mpcfile2("optim_obj.dat", ofstream::app);
							ofstream mpcfile3("optim_bal.dat", ofstream::app);
							mpcfile << mean_time << ", " << med_result << endl;
							mpcfile2 << med_result << ", ";
							mpcfile3 << *p_bal_res << ", ";
							mpcfile2.close();
							mpcfile3.close();
						}
						catch (int e)
						{
							mpcfile << "1, " << e;
							mpcfile.close();
							mpcfile2.close();
							mpcfile3.close();
							mpcfile4.close();
							exit(e);
						}
					}
				}
			}
			ofstream mpcfile2("optim_obj.dat", ofstream::app);
			ofstream mpcfile3("optim_bal.dat", ofstream::app);
			mpcfile2 << endl;
			mpcfile3 << endl;
			mpcfile2.close();
			mpcfile3.close();
		}
	}
}
