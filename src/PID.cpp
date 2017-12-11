#include "PID.h"
#include<iostream>
#include <math.h>
#include <chrono>

using namespace std;

PID::PID() {
  is_initialized = false;
  // change to true if tuning with Twiddle
  twiddle = false;
}

PID::~PID() {}

void PID::Init(double cte) {
  /**
   * Initialize PID Controller
   */
  
  // Set values of Pgain, Dgain, and Igain, respectively
  params.push_back(0.53);
  params.push_back(1.71);
  params.push_back(0.005);
  // Set d_gain values if using Twiddle;
  d_params.push_back(0.10);
  d_params.push_back(0.10);
  d_params.push_back(0.001);
  
  // second_try is used in Twiddle to decrease d_param if increasing or decreasing gain
  // did not improve performance
  second_try  = false;
  
  // cnt is used to determine how long to run before calculating error for Twiddle
  cnt = 0;
  
  // update_cnt determines which gain value to adjust during Twiddle
  update_cnt = 0;
  
  // Initialized best_error at -1 for Twiddle purposes
  best_err = -1;
  total_err = 0.0;

  prev_cte = cte;
  int_cte = 0.0;
  steer_angle = 0.0;
  
  is_initialized = true;
}

void PID::GetSteeringAngle(double cte, double speed) {
  /**
   * PID Controller
   */
  cnt+=1;
  cur_t = chrono::high_resolution_clock::now();
  // Since first change in time cannot be calculated at first step, approximate
  if (cnt==1) {
    dt = 0.015;
  }
  else {
    dt = chrono::duration_cast<chrono::duration<double>>(cur_t-prev_t).count();
  }
  int_cte += cte*dt;
  p_error = -params[0] * cte;
  i_error = -params[2] * int_cte;
  d_error = -params[1] * ((cte - prev_cte)/dt);
  prev_cte  = cte;
  steer_angle = (p_error + i_error + d_error);
  
  // Calculate avg. squared error over run for Twiddle
  total_err += (cte*cte) / cnt;
  if (cnt==5000) {
    // If tuning, run Twiddle
    if (twiddle) {
      Twiddle();
    }
    // Otherwise, reset count and int_cte, so integral doesn't explode
    else {
      cnt = 0;
      int_cte = 0;
    }
  }
  prev_t = cur_t;
}

void PID::GetThrottle(double cte, double speed) {
  /**
   * This decreases the throttle as CTE grows and visa versa with basic if-else statements.
   */
  
  // Ensure no division by 0
  speed = speed + 0.001;
  if (fabs(cte) > 0.6) {
    desired_speed = 40;
    speed_error = desired_speed - speed;
    throttle = speed_error / speed;
  }
  else if (fabs(cte) > 0.5) {
    desired_speed = 45;
    speed_error = desired_speed - speed;
    throttle = speed_error / speed;
  }
  else if (fabs(cte) > 0.4) {
    desired_speed = 55;
    speed_error = desired_speed - speed;
    throttle = speed_error / speed;
  }
  else if (fabs(cte) > 0.3) {
    desired_speed = 60;
    speed_error = desired_speed - speed;
    throttle = speed_error / speed;
  }
  else if (fabs(cte) > 0.2) {
    desired_speed = 70;
    speed_error = desired_speed - speed;
    throttle = speed_error / speed;
  }
  else {
    desired_speed = 75;
    speed_error = desired_speed - speed;
    throttle = speed_error / speed;
  }
}

void PID::Twiddle() {
  /**
   * Twiddle algorithm for tuning gain parameters.
   */
  if (best_err == -1) {
    cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
    cout << "d_params: " << d_params[0] << "     " <<  d_params[2] << "     " << d_params[1] << endl;
    cout << "Total Error " << total_err <<  endl;
    cout << "Best Error: " << best_err << endl;
    int_cte = 0;
    cnt  = 0;
    params[update_cnt] += d_params[update_cnt];
    best_err = total_err;
    total_err = 0.0;
    cout << "\nAfter 1, Now Updating Param: " << update_cnt << "..." <<endl;
  }
  else if (total_err < best_err) {
    cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
    cout << "d_params: " << d_params[0] << "     " <<  d_params[2] << "     " << d_params[1] << endl;
    cout << "Total Error " << total_err <<  endl;
    cout << "Best Error: " << best_err << endl;
    int_cte = 0;
    cnt  = 0;
    d_params[update_cnt] *= 1.1;
    update_cnt += 1;
    if (update_cnt == 3) {update_cnt = 0;}
    params[update_cnt] += d_params[update_cnt];
    best_err = total_err;
    total_err = 0.0;
    cout << "\nAfter 2, Now Updating Param: " << update_cnt << "..." <<endl;
  }
  else if (second_try) {
    cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
    cout << "d_params: " << d_params[0] << "     " <<  d_params[2] << "     " << d_params[1] << endl;
    cout << "Total Error " << total_err <<  endl;
    cout << "Best Error: " << best_err << endl;
    params[update_cnt] += d_params[update_cnt];
    d_params[update_cnt] *= 0.9;
    second_try = false;
    int_cte = 0;
    cnt  = 0;
    update_cnt += 1;
    if (update_cnt == 3) {update_cnt = 0;}
    params[update_cnt] += d_params[update_cnt];
    total_err = 0.0;
    cout << "\nAfter 3, Now Updating Param: " << update_cnt << "..." <<endl;
  }
  else {
    cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
    cout << "d_params: " << d_params[0] << "     " <<  d_params[2] << "     " << d_params[1] << endl;
    cout << "Total Error " << total_err <<  endl;
    cout << "Best Error: " << best_err <<  endl;
    params[update_cnt] -= 2*d_params[update_cnt];
    if (params[update_cnt] < 0) {params[update_cnt] = 0;}
    second_try = true;
    int_cte = 0;
    cnt  = 0;
    total_err = 0.0;
    cout << "\nAfter 4,Now Updating Param: " << update_cnt << "..." <<endl;
  }
}

