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


PID pid;
/*
 * When Debug is on , it prints outs some variables that can be montored
 */
int Debug ;
int main()
{
  uWS::Hub h;


  if(pid.IsInitialized == false)
  {
	  /*
	   * This flag can be used to turn on/off the auto tuning of PID gains
	   * false means optimization off
	   */
	  pid.IsOptimized =false;

	  double Kp_0,Ki_0,Kd_0;

	  Kp_0 = 0.138972;
	  Kd_0 = 0.615711;
	  Ki_0 = 0.00120251;
	  Debug = (pid.IsOptimized == true) ;
	  pid.Init(Kp_0,Ki_0,Kd_0);
	  pid.IsInitialized =true;
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


          //THis makes sure simulation is not reset on every run
          pid.Reset_Sim=0;

          /*
           * if Isoptimized is true, running twiddle, otherwise running PID with optimized gains
           */

          if (pid.IsOptimized ==false)
          {

        	  steer_value = pid.PID_Out(cte);
          }
          else
          {
        	  pid.Twiddle();
        	  steer_value = pid.PID_Out(cte);
          }

          // DEBUG
          if (Debug == 1 )
          {
			  std::cout << "CTE: " << cte << " Steering Value: " << steer_value <<std::endl;
			  std::cout<<"Kp:"<<pid.Kp<<" Kd:"<<pid.Kd<<" Ki:"<<pid.Ki<< std::endl;
			  std::cout<<"Reset_Sim:"<<pid.Reset_Sim<< "Phase:"<<pid.Phase<<std::endl;
			  std::cout<<"Cnts:"<<pid.Cnt<<"Tot_Cnt:"<<pid.Total_Cnts<<std::endl;
			  std::cout<<"dp[0]: "<<pid.dp[0]<< "dp[1]:"<<pid.dp[1]<< "dp[2]: "<<pid.dp[2]<<std::endl;
			  std::cout<<"icnt:"<<pid.icnt<< std::endl;

          }


          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // Resetting the simulation
          if (pid.Reset_Sim==1)
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
