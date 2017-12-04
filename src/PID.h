#ifndef PID_H
#define PID_H
#include<time.h>
#include<iostream>
#include<vector>
using namespace std;

class PID {
public:
  bool is_initialized;
  bool second_try;
  double steer_angle;
  int cnt;
  int update_cnt;
  double dt;
  double total_err;
  double best_err;
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  double int_cte;

  /*
  * Coefficients
  */
  vector<double> params;
  vector<double> d_params;

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
  void Init(double cte);

  /*
  * Update the PID error variables given cross track error.
  */
  void GetSteeringAngle(double cte, double angle, double speed);

  /*
  * Calculate the total PID error.
  */
  void Twiddle();
};

#endif /* PID_H */
