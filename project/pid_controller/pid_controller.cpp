/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  Kp = Kpi;
	Ki = Kii;
	Kd = Kdi;

	p_error = 0;
	i_error = 0;
	d_error = 0;

   lim_min = output_lim_mini;
   lim_max = output_lim_maxi;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  	double pre_cte = p_error;

	p_error = cte;
	i_error += cte * delta_time;
	d_error = (cte - pre_cte)/delta_time;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control =  -((Kp * p_error) - (Ki * i_error) - (Kd * d_error));

   if(control < lim_min)
      control = lim_min;
   else if (control >lim_max)
      control = lim_max;

    return control;

}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
   return delta_time;

}