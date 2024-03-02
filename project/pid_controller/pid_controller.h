/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
private:

    /*
    * Errors
    */
    double _cte, _cte_int, _cte_der;

    /*
    * Coefficients
    */
    double _Kp, _Ki, _Kd;

    /*
    * Output limits
    */
    double _output_lim_max, _output_lim_min;
  
    /*
    * Delta time
    */
    double _dt;

public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    void UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


