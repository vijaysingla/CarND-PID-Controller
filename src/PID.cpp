#include "PID.h"

using namespace std;



PID::PID()
{
Kp =0;
Ki =0;
Kd=0;
p_error =0;
i_error=0;
d_error =0;
IsInitialized =0;
IsOptimized =false;
Reset_Sim =0;
icnt =0;
Cnt =0;
icnt=0;
Phase =0;
dp[0] =0.06;
dp[1]= 0.001;
dp[2]=0.1;
Total_Cnts =400;
tol =0.12;
}

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
	return 0;
}

double PID::PID_Out(double cte)
{
	double steer_value;

	UpdateError(cte);
	// Setting the steer value by adding output from P,I and D controllers
	steer_value =-(Kp*p_error+Ki*i_error+Kd*d_error);

	// Ensuring that steering value remains between -1 and 1
    if (steer_value <= -1)
    {
 	 steer_value = -1;
    }
    else if (steer_value >=1)
    {
 	   steer_value = 1;
    }
    else
    {

    }

	return steer_value;
}

void PID::Twiddle()
{
    Cnt = Cnt +1;
    static double Best_Error;
    static double Error;
    static double Sum_dp;
    Sum_dp = dp[0]+dp[1]+dp[2];

    /*
     * if Sum_dp < tol, it means pid gains are optimized and terminating the twiddle algo
     * and using the optimized gains to run the simulator
     */

    if (Sum_dp < tol)
    {
    	IsOptimized = false;
    	Reset_Sim =1;
    	p_error =0;
    	d_error =0;
    	i_error=0;

    }

    /*
     *  Calculating the  error using initial P,I,D values
     *  This is categorized as Phase 0
     */
    if((Phase ==0) and (Cnt <= Total_Cnts)) //& (Cnt<=Total_Cnts)) //and )
    {


    	/*
    	 *  For first few seconds, cte error is not considered for calculating
    	 *  Best Error
    	 */
    	if (Cnt<= (Total_Cnts/2))
    	{
    		Best_Error =0;
    	}
    	else
    	{
    		Best_Error = Best_Error + (p_error*p_error);
    	}

    	/*
    	 *  when the cnt is equal total cnts, moving to phase 1 and resetting flags
    	 */
    	if (Cnt==Total_Cnts)
    	{
    		Best_Error = (Best_Error/Total_Cnts);
    		Phase = 1;
    		Cnt = 0;
    		Reset_Sim =1;
    		i_error  =0;
    		d_error=0;
    		i_error =0;
    	 }
    }
    if (((Phase==1) or (Phase==2)) and (Cnt<=Total_Cnts))
    {
    	/*
    	 *  Incrementing gains based on the value of dp for corresponding gain for
    	 *  Phase 1
    	 */

    	Kp = Kp + (dp[icnt]*((Phase==1) and (Cnt==1) and (icnt==0)));
    	Ki = Ki + (dp[icnt]*((Phase==1) and (Cnt==1) and (icnt==1)));
    	Kd = Kd + (dp[icnt]*((Phase==1) and (Cnt==1) and (icnt==2)));

    	/*
    	 * Accumulating Errors for phase 1 or phase 2 runs
    	 */
    	if (Cnt<= (Total_Cnts/2))
    	{
    		Error =0;
    	}
    	else
    	{
    		Error = Error+((p_error*p_error)/Total_Cnts);
    	}

    	if ((Cnt==Total_Cnts) and (Phase ==1))
    	{
    	  /*
    	   * If accumulated error for the run time = total_Cnts  is less than best error
    	   * increasing the dp value and resetting flags and incrementing icnt value
    	   */
    	  if (Error<Best_Error)
    	  {
    		  Best_Error = Error;
    		  Error = 0;
    		  dp[icnt] *=1.1;
    		  Cnt = 0;
    		  Reset_Sim =1;
    		  i_error  =0;
    		  d_error =0;
    		  p_error =0;
    		  icnt = (icnt+1)*(icnt<2);
    	  }
    	  /*
    	   * If accumulated error for the (run time = total_Cnts)  is greater than best error
    	   * decreasing the k value , moving to phase 2 and resetting flags
    	   */
    	  else
    	  {
    		  Kp -= (2 * dp[icnt]*(icnt==0));
    		  Ki -= (2 * dp[icnt]*(icnt==1));
    		  Kd -= (2 * dp[icnt]*(icnt==2));
    		  Error =0;
    		  Cnt = 0;
    		  Reset_Sim =1;
    		  i_error  =0;
    		  d_error =0;
    		  p_error =0;
    		  Phase =2;

    	  }
    	}

    	if ((Cnt==Total_Cnts) and (Phase ==2))
    	{
      	  /*
      	   * If accumulated error for the (run time = total_Cnts)  is less than best error
      	   * increasing the dp value and resetting flags, incrementing icnt and moving to phase 1
      	   */
    	  if (Error<Best_Error)
    	  {
    		  Best_Error = Error;
    		  Error =0;
    		  dp[icnt] *=1.1;
    		  Cnt = 0;
    		  Reset_Sim =1;
    		  Phase =1;
    		  i_error  =0;
    		  d_error =0;
    		  p_error =0;
    		  icnt = (icnt+1)*(icnt<2);
          }
      	  /*
      	   * If accumulated error for the (run time = total_Cnts)  is greater than best error
      	   * resetting kp value to previous optimized value, decreasing the dp value
      	   * and resetting flags, incrementing icnt and moving to phase 1
      	   */
          else
          {
        	  Kp += (dp[icnt]*(icnt==0));
         	  Ki += (dp[icnt]*(icnt==1));
         	  Kd += (dp[icnt]*(icnt==2));
         	  dp[icnt] *=0.9;
         	  Error =0;
         	  Cnt = 0;
         	  Reset_Sim =1;
			  i_error  =0;
			  d_error =0;
			  p_error =0;
			  Phase =1;
			  icnt = (icnt+1)*(icnt<2);
           }
    	}

    }
}






