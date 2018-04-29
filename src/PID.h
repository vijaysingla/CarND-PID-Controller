#ifndef PID_H

#define PID_H




class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
   * Initialization Flag
   */
  bool IsInitialized;
  /*
   * IsOptimized decides if twiddle algorithm is on or off
   */
  bool IsOptimized;
  /*
   * Flag that can be used to reset the simulation
   */
  bool Reset_Sim;
  /*
   * Cnt is used to keep track of no. of runs for the twiddle function
   */
  int Cnt;
  /*
   * icnt is used to decide whether kp, kd or ki is tuned
   */
  int icnt;
  /*
   * dp is array which holds the value of delta of gain for Kp,Kd and Ki
   */
  double dp[3];
/*
 * Phase is used to decide to change the value of dp and gains  depending on whether
 * error is decreasing or increasing
 */
  int Phase;

  /*
   * Total_Cnts corresponds to time used to run the car for optimization purpose
   */
  int Total_Cnts;

  /*
   * Based on "tol' value, twiddle algorithm is terminated and gains are optimized
   */

  double tol;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Calculate PID output
   */
  double PID_Out(double cte);

  /*
   * Twiddle Algorithm for gain auto tuning
   */
  void Twiddle();


};

#endif /* PID_H */
