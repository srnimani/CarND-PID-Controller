#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

//*********************************** New Variables *******************************************************

// Additional variables for twiddle
double p[3]    = {0.0, 0.0, 0.0} ; // Array containing tau_p, tau_i, tau_d
char   twid    = 'n'; // User input to decide whether to use twiddle or not

// Global variables for steering control to avoid twisty driving.. Used here is a ring buffer with
// on a first-in-first out basis.
double current_steer       = 0.0;
int  ring_buffer_index     = 0 ;
double* previous_steer     = NULL;   // Pointer to circular buffer containing previous steering values
int  buffer_size           = 20 ;

//************************************ End of New Variables ***********************************************


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


//***************************************** New Functions ***********************************************


void twiddle(PID pid, double cte)
// Twiddle algorithm
{
    // Twiddle variable declarations

    double dp[3]    = {1.0, 1.0, 1.0} ;
    double tw_err   = 0.0 ;
    double tol      = 0.0001 ;
    double sum      = dp[0] + dp[1] + dp[2] ;
    double best_err = abs(pid.TotalError()); // Take the absolute value of error
    
    p[0] = 0.0    ; // Kp
    p[1] = 0.0    ; // Ki
    p[2] = 0.0    ; // Kd
    
    pid.Init(p[0], p[1], p[2]);
    
    int iter = 0;

    while (sum > tol)
    {
        for (int i = 0; i < 3; i++)
        {
            // Twiddle Up
            p[i] += dp[i];
            pid.Kp = p[0];
            pid.Ki = p[1];
            pid.Kd = p[2];
            pid.UpdateError(cte);
            tw_err = abs(pid.TotalError()); // Take the absolute value of error
            
            if (tw_err < best_err)
            {
                best_err = tw_err;
                dp[i]    *= 1.1;
            }
            else
            {
                // Twiddle Down
                p[i]   -= 2*dp[i];
                pid.Kp = p[0];
                pid.Ki = p[1];
                pid.Kd = p[2];
                pid.UpdateError(cte);
                tw_err = abs(pid.TotalError()); // Take the absolute value of error
                
                if (tw_err < best_err)
                {
                    best_err = tw_err;
                    dp[i]    *= 1.1;
                }
                else
                {
                    p[i]   +=dp[i];
                    dp[i]  *= 0.9;
                }
            }
        }
        sum = dp[0] + dp[1] + dp[2] ;
        iter += 1;
    }
}

// Discrete sprrd control function to modulate the speed based on the streeing value

double discrete_speed_control(double steering_angle)
{
    double throttle_value = 0;
    
    if (abs(steering_angle) >= 0.8)
    {
        throttle_value = 0.1; // Slow down as the curve is sharp
    }
    else if ((abs(steering_angle) >= 0.6) && (abs(steering_angle) < 0.8))
    {
        throttle_value = 0.2;
    }
    else if ((abs(steering_angle) >= 0.6) && (abs(steering_angle) < 0.3))
    {
        throttle_value = 0.3;
    }
    else if ((abs(steering_angle) >= 0.3) && (abs(steering_angle) < 0.1))
    {
        throttle_value = 0.4;
    }
    else
    {
        throttle_value = 0.5; // Almost straight..
    }
    return throttle_value ;
}



// The below function uses an analogous speed control.. not used as it is seen to be less effective
// than the discrete speed control

double linear_speed_control(double steering_angle)
{
    double throttle_value;
    
    throttle_value = (1 - abs(steering_angle) * 0.2 + 0.1);
    
    return throttle_value ;
}


// Check for steering out of bouns.. Steering should be between +1 and -1

double steering_check(double current_steer)
{
    
    if (current_steer > 1.0)
    {
        current_steer = 1.0;
    }
    if (current_steer < -1.0)
    {
        current_steer = -1.0;
    }
    
    return current_steer ;
}


// The below function is a circular buffer implementation to take the average of the last 'n' steering
// angles. Every run, the one at the index pointer replaced with the current value received
// and the average value returned.. Currently not used as it is found to be less effective than
// the simple control

double steering_control(double current_steer)
{

     double new_steer = 0 ;

     // Replace the the oldest value in the circular buffer with the new one
     
     previous_steer[ring_buffer_index] = steering_check(current_steer);
    
     // Loop to calculate the average of last n (= buffer_size) steering angles
     
     for (int i =0; i < buffer_size; ++i)
     {
     new_steer += previous_steer[i];
     }
     
     new_steer /= buffer_size ;
     
     // Advance the pointer to write the next value, note it is a circular buffer
     
     ring_buffer_index = (ring_buffer_index + 1) % buffer_size ;
     
     cout << "current\t" << current_steer << "new  = \t" << new_steer << endl;
     
     return new_steer ;
    
}

//***************************************** End of New Functions ****************************************


int main()
{
  uWS::Hub h;

  PID pid;
    
  // TODO: Initialize the pid variable.
    
  // Initialize Kp, Ki and Kd for non-twiddle case
    
  p[0] = 0.3    ; // Kp
  p[1] = 0.001  ; // Ki
  p[2] = 4.0    ; // Kd
    
  pid.Init(p[0], p[1], p[2]);

  /*
  previous_steer = new double[buffer_size];
    
  for (int i=0; i < buffer_size; i++)
  {
    previous_steer[i] = 0;    // Initialize all elements to zero.
  }
   
  */

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
          
          pid.UpdateError(cte);
            
          twiddle(pid, cte); // Twiddle the parameters to get the best values

          current_steer = -pid.TotalError();
      
          steer_value = steering_check(current_steer) ; //
            
          // DEBUG
          cout << "CTE: " << cte << " Steering Value: " << steer_value << endl;
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
            
          // Speed control based on steering angle
            
          double throttle_value = discrete_speed_control(steer_value);
            
          msgJson["throttle"]   = throttle_value; //
            
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          cout << msg << endl;
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
