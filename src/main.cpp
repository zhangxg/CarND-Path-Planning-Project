#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// the x, y are the location of the car. 
// the mapx and mapy are the map points list.

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}


// how to write comments; google comment code style: https://google.github.io/styleguide/cppguide.html#Comments

// the x, y are the car's location. 
// what's the theta?
// todo: explain this with a graph.
// todo: why the next way points is calcualted? by considering the heading direction of the car, this will get the next way point the car will drive to, this is necessary because the nearest point may be behin the car, and we don't want the car to drive back. 


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

  // this is the car's driving direction. 
	double heading = atan2((map_y-y), (map_x-x));

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
// todo: explain how? 
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() 
{
  uWS::Hub h;


  // todo: moving the file parsing to a function, maybe a file. 
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // xg: the socket package 
  int lane = 1;

  // to avoid jerk, this needs to be set to zero, 
  // accelerate from zero, with 0.224 for each step;
  // double ref_vel = 49.5;
  double ref_vel = 0.0;

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        // xg: for now, the simulator gives us the localization data, 
        // in the practice, this will de determined by localization modular.  
        if (event == "telemetry") {
          // j[1] is the data JSON object
        	// Main car's localization Data
        	double car_x = j[1]["x"];
        	double car_y = j[1]["y"];
        	double car_s = j[1]["s"];
        	double car_d = j[1]["d"];
        	double car_yaw = j[1]["yaw"];
        	double car_speed = j[1]["speed"];

        	// Previous path data given to the Planner
        	auto previous_path_x = j[1]["previous_path_x"];
        	auto previous_path_y = j[1]["previous_path_y"];
        	// Previous path's end s and d values 
        	double end_path_s = j[1]["end_path_s"];
        	double end_path_d = j[1]["end_path_d"];

        	// Sensor Fusion Data, a list of all other cars on the same side of the road.
        	auto sensor_fusion = j[1]["sensor_fusion"];

          // =====  above is the provided code 

          int prev_size = previous_path_x.size();

          // if there are remaining point in previous path, 
          // we set the car's s to the last waypoint's s
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }

          bool too_close = false;

          for (int i = 0; i < sensor_fusion.size(); ++i)
          {
            float d = sensor_fusion[i][6];
            if (d > (2+4*lane-2) && d < (2+4*lane+2)) // d in (4, 8), which is lane 1; 
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // here we are using the remaining points of "my car" to calculate the other car's predication s; since each points is 0.02 seconds, we mutiple it by the speed of the car. 
              check_car_s += ((double)prev_size*0.02*check_speed);

              // here we only care about the car in front of us, 
              // with the condition check of check_car_s > car_s
              if (check_car_s > car_s && (check_car_s - car_s) < 30)
              {
                // with this set, i saw a jerk in the simulator, 
                // this happens because the speed is suddenly changed from 
                // 49.5 to 29.5. 
                // ref_vel = 29.5;
                too_close = true;
                // if it's close, we blindly change to the left lane. 
                if (lane > 0)
                {
                  lane = 0;
                }
              }
            }
          }

          if (too_close)
          {
            ref_vel -= 0.224;
          }
          else if (ref_vel < 49.5)
          {
            ref_vel += 0.224;
            // here i want to change lane back to 1, 
            // found that the car starts quite slow.
            // lane = 1;
          }

          std::vector<double> ptsx;
          std::vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          //xg-todo: when to convert to radius, when to use degree? 
          double ref_yaw = deg2rad(car_yaw);
          // cout << "the car's parameter" << endl;
          // cout << car_x << ", " << ", " << car_y << ", " << car_yaw << endl;

          // the points are used to generate the spline, 
          // if the value is less than 2 (zero or 1), it's hard to make the 
          // the spline. we at least give it two. 
          // cout << "vaues in list " << prev_size << endl;
          if (prev_size < 2) 
          {
            // xg-todo: use degree
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // if the returned previous path contains more than 2 points, 
          // we will leverage them. 
          else 
          {
            // the last point the car used. 
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            // get the next last point used by the car
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            // get the car's heading (angel) 
            // but it's not used ? 
            // xg-todo: that's the atan2() function, it used a lot;
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          vector<double > next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double > next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double > next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);


          //transfer the cartisian coordinates to the car's coordinates
          for (int i = 0; i < ptsx.size(); ++i)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            double car_angel = 0 - ref_yaw;
            ptsx[i] = shift_x * cos(car_angel) - shift_y * sin(car_angel);
            ptsy[i] = shift_x * sin(car_angel) + shift_y * cos(car_angel);
          }

          // introduce the spline
          tk::spline s;

          s.set_points(ptsx, ptsy);

          // the new waypoints sent to the car
        	vector<double> next_x_vals;
        	vector<double> next_y_vals;

          // reuse the previous path;
          // don't the already be used? not, they are the leftovers. 
          // the simulator makes sure only the leftovers returned.
          for (int i = 0; i < previous_path_x.size(); ++i)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate the points from the spline
          double target_x = 30.0;
          double target_y = s(target_x);
          // we used the car's coordinates
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          for (int i = 0; i <= 50 - previous_path_x.size(); ++i)
          {
            double N = target_dist / (0.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            // for next use
            x_add_on = x_point;

            // x_point and y_point are in car's coordinate
            double x_ref = x_point;
            double y_ref = y_point;

            // convert back to normal coordinates
            // rotate back
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            // translate back
            // bug: if set this way, the car doen't move.  see: 
            // https://discussions.udacity.com/t/walkthrough-video-code-problem/376665/7

            // x_point += x_ref;
            // y_point += y_ref;


            x_point += ref_x;
            y_point += ref_y;

            //add the new way points
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // cout << " the points in the list " << endl;
          // for (int i = 0; i < next_x_vals.size(); ++i)
          // {
          //   cout << next_x_vals[i] << ", " << next_y_vals[i] << endl;
          // }

          // cout << next_x_vals.size() << endl;
        	// // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
         //  //xg: getting started code:
         //  double dist_inc = 0.5;
          
         //  // std::vector<double> f_start = getFrenet(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y); 
         //  // cout << f_start[0] << "," << f_start[1] << endl;
         //  // std::vector<double> c_xy; 
         //  // cout << "moving ... " << endl;

         //  for(int i = 0; i < 50; i++)
         //  {
         //    // 1.  driving a straight line
         //    // next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
         //    // next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));

         //    // 2. stay in the line, using frenet coordinates
         //    // the getXY() methods requires s, x, y, not x, y, s, the later can not move. 
         //    // this now moves, but osscilate a lot. 
         //    // c_xy = getXY(f_start[0]+dist_inc*i, f_start[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);

         //    // next_x_vals.push_back(c_xy[0]);
         //    // next_y_vals.push_back(c_xy[1]);

         //    // above my work, not working. why? 
         //    double next_s = car_s + (i + 1) * dist_inc;
         //    double next_d = 6;
         //    std::vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

         //    next_x_vals.push_back(xy[0]);
         //    next_y_vals.push_back(xy[1]);              
         //  }
          // cout << next_x_vals[0] << ", " << next_y_vals[0] << endl;
          // STOP
          json msgJson;
        	msgJson["next_x"] = next_x_vals;
        	msgJson["next_y"] = next_y_vals;

        	auto msg = "42[\"control\","+ msgJson.dump()+"]";

        	//this_thread::sleep_for(chrono::milliseconds(1000));
        	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}


/**
========================
the errors:
variable 'lane' cannot be implicitly captured in a lambda with no capture-default specified
          vector<double > next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypo...

https://stackoverflow.com/questions/30217956/error-variable-cannot-be-implicitly-captured-because-no-default-capture-mode-h/30217989


========================

*/
