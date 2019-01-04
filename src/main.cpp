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




int main(int argc, char *argv[])
{
    /*
    if (argc < 4){
        std::cerr << "\nInput error! Enter P I D constants as arguments:\n";
        std::cerr << "e.g.: " << argv[0] << " 0.1 0.002 3.5\n" << std::endl;
        exit(1);
    }
     */
    
    uWS::Hub h;
    
    // Initialize the pid variable.
    PID pid;
    
    // pass arguments 2,3 & 4 as values for PID constants
    //double const user_Kp = atof(argv[1]);
    //double const user_Ki = atof(argv[2]);
    //double const user_Kd = atof(argv[3]);
    
    // Initialize the pid variables. P,I,D
    
    // steering PID init taking arguments from command line
    //pid.Init(user_Kp, user_Ki, user_Kd);
    // tuned parameters for submission
    pid.Init(0.1,0.002,3.5);
  
    // Results using Twiddle
    //Kp: 1.14981 Ki: 0.431999 Kd: 2.3973
    //pid.Init(1.14981,0.431999,2.3973);

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
          double throttle_value;
            
            

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
            
            // update total error
            pid.UpdateError(cte);
            
            
            steer_value = pid.TotalError(cte);
            
            // control thottle based on steering request
            if (fabs(steer_value) > 0.4 ) {
                throttle_value = -0.1; // BRAKE
            } else {
                throttle_value = 0.45;
            }
            
            std::cout <<  "throttle:" << throttle_value << "\tSTRG: " << angle << "\tSPEED: " << speed << std::endl;
            
            // limit steering changes if greater than limits
            if(steer_value < -1.0) {
                steer_value = -0.7;
            }
            else if(steer_value > 1.0) {
                steer_value = 0.7;
            }
          
          // DEBUG
            
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

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
