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
double refvel = 0;
int lane = 1;
int lane_change_cooldown = 0;
bool too_close = false;

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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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
bool myfn(pair<double,double> i, pair<double,double> j) { return i.second<j.second; }
bool myfn2(double i, double j) { return i<j; }


string getNextState(vector< vector<double> > sensor_fusion, double car_s, double car_end_s, int lane, int prev_size) {
  if (!too_close)
    return "KL";

  /*
   * this code is heavily borrowed from the behavior planning quiz:
   *
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right (Unimplemented)
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    */
  vector<pair<double,double>> lane_speeds;
  for (int i = 0 ; i < 3; i++) {
    lane_speeds.push_back(make_pair(-1,500));
  }

  vector<double> lane_collision_cost;
  for (int i =0;i<3;i++) {
    lane_collision_cost.push_back(0);
  }
  bool is_blocked_on_right = false;
  bool is_blocked_on_left = false;
  double lane_watch_s = 20;
  double fudge_s = 10;
  for (auto sens : sensor_fusion) {
    double speeds = 0;
    int i = sens[0];
    double vx = sens[3];
    double vy = sens[4];
    double s = sens[5];
    double d = sens[6];
    double car_speed = sqrt(vx * vx + vy * vy);
    double end_check_s = s + prev_size * 0.02 * car_speed;

    int car_lane = floor(d / 4);
    bool in_s_range = fabs(end_check_s - car_end_s - fudge_s) < lane_watch_s; //safety net
    bool close_enough = end_check_s > car_end_s && (end_check_s - car_end_s < 100);
    if (car_lane >= 0 && car_lane <= 2) {
      if (car_lane == (lane + 1) && in_s_range) {
        cout << "Blocked on the right cuz car in lane " << car_lane << " at s: " << s << " ecs: " << end_check_s << " end_s: " << car_end_s << endl;
        is_blocked_on_right = true;
        lane_collision_cost[car_lane] = 1;
      } else if (car_lane == (lane - 1 ) && in_s_range) {
        cout << "Blocked on the left cuz car in lane " << car_lane << " at s: " << s << " ecs: " << end_check_s << " end_s: " << car_end_s << endl;
        is_blocked_on_left = true;
        lane_collision_cost[car_lane] = 1;
      } else {
        lane_collision_cost[car_lane] = 0;
      }

      if (close_enough && (lane_speeds[car_lane].first > end_check_s || lane_speeds[car_lane].first == -1)) {
        lane_speeds[car_lane] = std::make_pair(end_check_s, car_speed);
      }
    }
  }

  auto mx_speed = *max_element(lane_speeds.begin(), lane_speeds.end(), myfn);
  auto mn_speed = *min_element(lane_speeds.begin(), lane_speeds.end(), myfn);
  double max_speed = mx_speed.second;
  double min_speed = mn_speed.second;
  std::vector<double> lane_speed_cost;
  lane_speed_cost.reserve(3);
  if (max_speed > 0) {
    for (int i = 0; i < 3; i++) {
      lane_speed_cost[i] = (max_speed - lane_speeds[i].second + min_speed) / max_speed;
//      cout << "speedcost on lane: " << to_string(i) << " : " << to_string(lane_speed_cost[i]) << endl;
    }
  } else {
    return "KL";
  }

  std::vector<double> lane_cost;
  for (int i = 0; i < 3; i++) {
    double lcost = lane_collision_cost[i] == 0 ? lane_speed_cost[i] : 1.0;
    lane_cost.push_back(lcost);
//    cout << "Lane " << to_string(i) << " Cost: " << to_string(lane_cost[i]) << " ccost:" << to_string(lane_collision_cost[i]) << endl;
  }
  int min_cost_lane = distance(lane_cost.begin() ,min_element(lane_cost.begin(), lane_cost.end(), myfn2));
  string state;
  if (min_cost_lane == lane) {
    state = "KL";
  } else if (min_cost_lane > lane) {
    if (is_blocked_on_right)
      state = "PLCR"; // Basically KL
    else
      state = "LCR";
  } else {
    if (is_blocked_on_left)
      state = "PLCL"; // Basically KL
    else
      state = "LCL";
  }
  cout << "Current lane " << lane << " best lane: " << min_cost_lane << " State : " << state << endl;
  return state;

}
double getTargetVelocityInLane(vector<vector<double>> sensor_fusion, double car_end_s, int prev_size, int lane, double ref_vel, double max_velocity, double target_acc) {

  double car_speed = 0;
  for (auto sens : sensor_fusion) {
    double vx = sens[3];
    double vy = sens[4];
    double s = sens[5];
    double d = sens[6];

     car_speed = sqrt(vx*vx + vy*vy); //convert to MPH
    double car_speed_mph = car_speed * 2.237;
    double end_check_s = s + prev_size*0.02*car_speed;
    if (d > (4*lane) && d < (4+4*lane)) {
      //car is in yo lane
      if (end_check_s > car_end_s){
        if ((end_check_s - car_end_s) < 30) {
          too_close = true;
//          cout << "First close encounter, decreasing speed" << endl;
          return fmax(fmax(0, car_speed_mph-2), ref_vel - target_acc * 0.02); //tracks the car nicely
        } else if (too_close && (end_check_s - car_end_s) < 40) {
//          cout << "Tracking car. car speed:" << car_speed_mph<< " want speed: " << (ref_vel + target_acc * 0.02) << endl;
          double new_speed = ref_vel + target_acc * 0.02;
          return new_speed > car_speed_mph ? car_speed_mph : new_speed;
        }
      }
    }
  }

  if (too_close) {
    cout << "Aaah Wide open road.." << endl;
    too_close  = false;
  }


  return fmin(max_velocity, ref_vel + target_acc * 0.02);
}

int main() {
  uWS::Hub h;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            double dist_inc = 0.5;
            int psize = previous_path_x.size();
            vector<double> x_val;
            vector<double> y_val;
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            if (previous_path_x.size() < 2) {
              x_val.push_back(car_x - cos(ref_yaw));
              y_val.push_back(car_y - sin(ref_yaw));
              x_val.push_back(car_x);
              y_val.push_back(car_y);
            } else {
              ref_x = previous_path_x[psize-1];
              ref_y = previous_path_y[psize-1];
              double prev_ref_x = previous_path_x[psize-2];
              double prev_ref_y = previous_path_y[psize-2];
              ref_yaw = atan2(ref_y - prev_ref_y,
                              ref_x - prev_ref_x);
              x_val.push_back(prev_ref_x);
              y_val.push_back(prev_ref_y);

              x_val.push_back(ref_x);
              y_val.push_back(ref_y);
            }

            lane_change_cooldown = max(0, lane_change_cooldown - (50-psize));
            if (!lane_change_cooldown) {
              string state = getNextState(sensor_fusion, car_s, end_path_s, lane, psize);
              if (state == "LCR") {
                lane_change_cooldown = 50*5;
                lane += 1;
              } else if (state == "LCL") {
                lane_change_cooldown = 50*5;
                lane -= 1;
              }
            }
            int lane_width = 4;
            //Add frenet predictions:
            for (int i = 1; i < 4; i++) {
              vector<double> xy  = getXY(car_s + i * 50, lane_width/2 + lane * lane_width, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              x_val.push_back(xy[0]);
              y_val.push_back(xy[1]);
            }

            for (int i = 0; i < x_val.size(); i++) {

              double shift_x = x_val[i] - ref_x;
              double shift_y = y_val[i] - ref_y;
              x_val[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
              y_val[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
            }

            bool bad_points = false;
            if (!((x_val[0] < x_val[1]) && (x_val[1] < x_val[2]))) {
              bad_points = true; // so this is a terrible hack but I have no idea how to do better.
              // if true then this iteration is wasted.
            }

            //get ready for splinin

            for (int i = 0; i < previous_path_x.size(); i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

          if (!bad_points) {

            tk::spline sp;
            sp.set_points(x_val, y_val);

            double target_x = 30;
            double target_y = sp(target_x);
            double target_dist = sqrt((target_x * target_x) + target_y * target_y);
            double add_on = 0;
            refvel = getTargetVelocityInLane(sensor_fusion, psize > 0 ? end_path_s : car_s, psize, lane, refvel, 45,
                                             20);

            double N = target_dist / (0.02 * refvel / 2.24);

            for (int i = 0; i < (50 - previous_path_x.size()); i++) {
              double x_spline = add_on + target_x / N;
              double y_spline = sp(x_spline);
              add_on = x_spline;
              double x_rot = x_spline * cos(ref_yaw) - y_spline * sin(ref_yaw);
              double y_rot = x_spline * sin(ref_yaw) + y_spline * cos(ref_yaw);
              x_rot += ref_x;
              y_rot += ref_y;
              next_x_vals.push_back(x_rot);
              next_y_vals.push_back(y_rot);
              //cout << x_rot << "," << y_rot << " | ";

            }
          }
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
















































































