#ifndef PID_H
#define PID_H
#include<time.h>
#include<iostream>
#include<vector>
using namespace std;

class PID {
public:
  /*
   * Booleans for initialization, whether to run Twiddle, and which Twiddle step
   */
  bool is_initialized;
  bool twiddle;
  bool second_try;
  
  /*
   * cnt and update cnt for Twiddle
   */
  int cnt;
  int update_cnt;
  
  /*
   * change in time between messages from simulator
   */
  double dt;
  
  /*
   * Steer angle from PID controller
   */
  double steer_angle;
  
  /*
   * Throttle
   */
  double throttle;
  
  /*
   * Desired speed based on CTE and speed error
   */
  double desired_speed;
  double speed_error;
  
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;
  double int_cte;
  double total_err;
  double best_err;

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
  * Udate Steering angle with PID Controller.
  */
  void GetSteeringAngle(double cte, double speed);
  
  /*
   * Update throttle based on current CTE
   */
  void GetThrottle(double cte, double speed);

  /*
  * Twiddle algorithm for tuning.
  */
  void Twiddle();
};

#endif /* PID_H */
