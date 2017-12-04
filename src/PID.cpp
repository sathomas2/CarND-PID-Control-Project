#include "PID.h"
#include<iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  is_initialized = false;
}

PID::~PID() {}

void PID::Init(double cte) {
  params.push_back(0.49); // 0.35
  params.push_back(60.0); // 40.0
  params.push_back(0.0002); // 0.0003
  d_params.push_back(0.1);
  d_params.push_back(5.0);
  d_params.push_back(0.0001);
  prev_cte = cte;
  int_cte = 0;
  steer_angle = 0.0;
  is_initialized = true;
  second_try  = false;
  cnt = 0;
  update_cnt = 0;
  dt = 0.02;
  total_err = 0.0;
  best_err = -1;
  cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
  
}

void PID::GetSteeringAngle(double cte, double angle, double speed) {
  cnt+=1;
  //cout << "Angle: " << angle << endl;
  //double distance = (speed * 1000.0 / 3600.0) * dt;
  //cout << "DISTANCE: " << distance << endl;
  int_cte += cte;
  p_error = -params[0] * cte;
  i_error = -params[2] * int_cte;
  d_error = -params[1] * (cte - prev_cte);
  //cout << "P-error " << p_error <<endl;
  //cout << "I-error " << i_error <<endl;
  //cout << "D-error " << d_error <<endl;
  prev_cte  = cte;
  steer_angle = (p_error + i_error + d_error);
  total_err += (cte*cte) / cnt;
  if (cnt==5000) {
    //cnt = 0;
    //int_cte = 0;
    cout << "Total Error " << total_err <<  endl;
    cout << "Best Error: " << best_err << "\n" << endl;
    Twiddle();
  }
}

void PID::Twiddle() {
  if (best_err == -1) {
    best_err = total_err;
    total_err = 0.0;
    int_cte = 0;
    cnt  = 0;
    params[update_cnt] += d_params[update_cnt];
    cout << "After 1, Update Param: " << update_cnt << endl;
    cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
    cout << "d_params: " << d_params[0] << "     " <<  d_params[2] << "     " << d_params[1] << endl;

  }
  else if (total_err < best_err) {
    best_err = total_err;
    total_err = 0.0;
    int_cte = 0;
    cnt  = 0;
    d_params[update_cnt] *= 1.1;
    update_cnt += 1;
    if (update_cnt == 3) {update_cnt = 0;}
    params[update_cnt] += d_params[update_cnt];
    cout << "After 2, Update Param: " << update_cnt << endl;
    cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
    cout << "d_params: " << d_params[0] << "     " <<  d_params[2] << "     " << d_params[1] << endl;
  }
  else if (second_try) {
    params[update_cnt] += d_params[update_cnt];
    d_params[update_cnt] *= 0.9;
    second_try = false;
    total_err = 0.0;
    int_cte = 0;
    cnt  = 0;
    update_cnt += 1;
    if (update_cnt == 3) {update_cnt = 0;}
    params[update_cnt] += d_params[update_cnt];
    cout << "After 3, Update Param: " << update_cnt << endl;
    cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
    cout << "d_params: " << d_params[0] << "     " <<  d_params[2] << "     " << d_params[1] << endl;
  }
  else {
    params[update_cnt] -= 2*d_params[update_cnt];
    if (params[update_cnt] < 0) {params[update_cnt] = 0;}
    second_try = true;
    total_err = 0.0;
    int_cte = 0;
    cnt  = 0;
    cout << "After 4, Update Param: " << update_cnt << endl;
    cout << "P: " << params[0] << "  I: " << params[2] << "  D: " << params[1] << endl;
    cout << "d_params: " << d_params[0] << "     " <<  d_params[2] << "     " << d_params[1] << endl;
  }
  
}

