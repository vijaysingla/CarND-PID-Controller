#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int IsInitialized = 0 ;
PID pid;
int Phase;
int Cnt;
int Reset_Sim = 0;
double Best_Error;
int Total_Cnts;
double dp[3];
double tol;
int IsTwiddle;
int icnt =0;
double Error=0;
int IsOptimized =1;
int main()
{
  uWS::Hub h;

  // TODO: Initialize the pid variable.
  if(IsInitialized !=1)
  {
	  double Kp_0,Ki_0,Kd_0;
	  if (IsOptimized != 1)
	  {
		  Kp_0 = 0.157667;
		  Kd_0 =1.45305;
		  Ki_0 = 0;
	  }
	  else
	  {
		  Kp_0 =  0.157667;//0.235734; //0.201039;
		  Kd_0 = 1.45305; //1.14614;//0.594921;
		  Ki_0 = 0.000381473;
	  }


	  pid.Init(Kp_0,Ki_0,Kd_0);
	  IsInitialized = 1;
	  Phase = 0;
	  Cnt =0;
	  dp[0] =0.06;
	  dp[1]= 0;
	  dp[2]=0.1;
	  tol = 0.07;
	  Total_Cnts =300;

  }

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          Reset_Sim=0;
          IsTwiddle = (1*((dp[0]+dp[1]+dp[2])>tol))*((IsOptimized==0));

          if (IsTwiddle ==1)
          {
              Cnt = Cnt +1;
              if((Phase ==0) and (Cnt<=Total_Cnts) )
              {
            	  std::cout<<"Cnt in loop: "<<Cnt<<std::endl;
            	  pid.UpdateError(cte);
            	  steer_value =pid.TotalError();
            	  Best_Error = Best_Error + (cte*cte);
            	  if (Cnt==Total_Cnts)
            	  {
            		  Best_Error = (Best_Error/Total_Cnts);

            		  Phase = 1;
            		  Cnt = 0;
            		  Reset_Sim =1;
            		  pid.i_error  =0;
            	  }
              }


              if (((Phase==1) or (Phase==2)) and (Cnt<=Total_Cnts))
              {
            	  if (((Phase==1)) and (Cnt==1) and (icnt==0))
            	  {
            		  std::cout<<"changing Kp"<<std::endl;
            		  pid.Kp = pid.Kp + dp[icnt];
            	  }
            	  else if (((Phase==1)) and (Cnt==1) and (icnt==1))
            	  {
            		  pid.Ki = pid.Ki +dp[icnt];
            	  }
            	  else if(((Phase==1)) and (Cnt==1)and (icnt==2))
            	  {
            		  pid.Kd = pid.Kd +dp[icnt];
            	  }
            	  pid.UpdateError(cte);
            	  steer_value =pid.TotalError();

            	  Error = Error+((cte*cte)/Total_Cnts);

            	  if ((Cnt==Total_Cnts) and (Phase ==1))
            	  {

            		  if (Error<Best_Error)
            		  {
            			  Best_Error = Error;
            			  Error = 0;
            			  dp[icnt] *=1.1;
            			  Cnt = 0;
            			  Reset_Sim =1;
            			  pid.i_error  =0;
            			  pid.d_error =0;
            			  pid.p_error =0;
            			  icnt = (icnt+1)*(icnt<2);
            		  }
            		  else
            		  {
            			  pid.Kp -= (2 * dp[icnt]*(icnt==0));
            			  pid.Ki -= (2 * dp[icnt]*(icnt==1));
            			  pid.Kd -= (2 * dp[icnt]*(icnt==2));
            			  Error =0;
            			  Cnt = 0;
            			  Reset_Sim =1;
            			  pid.i_error  =0;
            			  pid.d_error =0;
            			  pid.p_error =0;
            			  Phase =2;

            		  }
            	  }
            	  if ((Cnt==Total_Cnts) and (Phase ==2))
            	  {

            		  if (Error<Best_Error)
            		  {
            			  Best_Error = Error;
            			  Error =0;
            			  dp[icnt] *=1.1;
            			  Cnt = 0;
            			  Reset_Sim =1;
            			  Phase =1;
            			  pid.i_error  =0;
            			  pid.d_error =0;
            			  pid.p_error =0;
            			  icnt = (icnt+1)*(icnt<2);
            		  }

            		  else
            		  {
            			  pid.Kp += (dp[icnt]*(icnt==0));
            			  pid.Ki += (dp[icnt]*(icnt==1));
            			  pid.Kd += (dp[icnt]*(icnt==2));
            			  dp[icnt] *=0.9;
            			  Error =0;
               			  Cnt = 0;
               			  Reset_Sim =1;
               			  pid.i_error  =0;
            			  pid.d_error =0;
            			  pid.p_error =0;
               			  icnt = (icnt+1)*(icnt<2);
               			  Phase =1;
            		  }
            	  }
          }

          }

          else
          {

           pid.UpdateError(cte);
           steer_value =pid.TotalError();
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
           std::cout<<"optimized"<<std::endl;

           }


          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value <<std::endl;
//          "angle:"<<angle
//        		  << "speed:"<<speed<<std::endl;
          std::cout<<"Kp:"<<pid.Kp<<" Kd:"<<pid.Kd<<" Ki:"<<pid.Ki<< std::endl;
          std::cout<<"Reset_Sim:"<<Reset_Sim<< "Phase:"<<Phase<<std::endl;
          std::cout<<"Cnts:"<<Cnt<<"Tot_Cnt:"<<Total_Cnts<<std::endl;
          std::cout<<"dp[0]: "<<dp[0]<< "dp[1]:"<<dp[1]<< "dp[2]: "<<dp[2]<<std::endl;
          std::cout<< "Best_Error: "<<Best_Error <<"Error:"<<Error<<"icnt:"<<icnt<< std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          if (Reset_Sim==1)
          {
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
