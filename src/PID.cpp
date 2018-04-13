#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	this->Kp = Kp;
	this->Ki =Ki;
	this->Kd = Kd;
	//'this' is a pointer which can be used to get the object's parameters like Kp hidden by local variable
	p_error =  0;
	i_error  = 0;
	d_error  = 0;
}

void PID::UpdateError(double cte) {
	double prev_error;
	prev_error =p_error;
	p_error = cte;
	i_error = i_error + cte;
	d_error =  cte - prev_error;

}

double PID::TotalError() {
	double total_error;
	total_error = Kp*p_error+Ki*i_error+Kd*d_error;
	return   total_error;

}

