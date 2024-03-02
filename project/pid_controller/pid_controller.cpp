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
   _Kp = Kpi;
   _Ki = Kii;
   _Kd = Kdi;
   _output_lim_max = output_lim_maxi;
   _output_lim_min = output_lim_mini;

   _cte = 0;
   _cte_int = 0;
   _cte_der = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  _cte_der = (cte - _cte)/_dt;
  _cte_int += cte*_dt;
  _cte = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = _Kp*_cte + _Ki*_cte_int + _Kd*_cte_der;
    control = min(max(control, _output_lim_min), _output_lim_max);
    return control;
}

void PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  _dt = new_delta_time;
}