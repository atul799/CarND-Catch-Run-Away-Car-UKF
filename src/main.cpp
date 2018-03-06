#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "ukf.h"

#include <vector>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  //////////////////////////////////
  // open a file handle to write results for visualization
  //string out_file_name="../outputs/UKF_fwd_high_a_yawdd.out";
  string out_file_name="../outputs/UKF_runaway.out";
  //string out_file_name="../outputs/UKF_fwd_low_a_yawdd_laser_unscented.out";
  //string out_file_name="../outputs/UKF_rev.out";
  //string out_file_name="../outputs/UKF_laseronly.out";
  //string out_file_name="../outputs/UKF_radaronly.out";
  ofstream out_file (out_file_name, ofstream::out);
  if (!out_file.is_open())  {
	  cerr << "Cannot open output file: " << out_file_name << endl;
	  exit(EXIT_FAILURE);
  }

  // Create a UKF instance
  UKF ukf;
  
  double target_x = 0.0;
  double target_y = 0.0;

  //vectors to capture heading dir and distance difference
  //vector<float> heading_dir_diff;
  //vector<float> heading_dist_diff;
  float heading_dir_diff[5]={0,0,0,0,0};
  float heading_dist_diff[5]={0,0,0,0,0};
  int mm=0;

  h.onMessage([&ukf,&target_x,&target_y,&out_file,&heading_dir_diff,&heading_dist_diff,&mm](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") {
      	
      	
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          double hunter_x = std::stod(j[1]["hunter_x"].get<std::string>());
          double hunter_y = std::stod(j[1]["hunter_y"].get<std::string>());
          double hunter_heading = std::stod(j[1]["hunter_heading"].get<std::string>());
          
          string lidar_measurment = j[1]["lidar_measurement"];
          
          MeasurementPackage meas_package_L;
          istringstream iss_L(lidar_measurment);
    	  long long timestamp_L;

    	  // reads first element from the current line
    	  string sensor_type_L;
    	  iss_L >> sensor_type_L;

      	  // read measurements at this timestamp
      	  meas_package_L.sensor_type_ = MeasurementPackage::LASER;
          meas_package_L.raw_measurements_ = VectorXd(2);
          float px;
      	  float py;
          iss_L >> px;
          iss_L >> py;
          meas_package_L.raw_measurements_ << px, py;
          iss_L >> timestamp_L;
          meas_package_L.timestamp_ = timestamp_L;
          
    	  ukf.ProcessMeasurement(meas_package_L);
		 
    	  string radar_measurment = j[1]["radar_measurement"];
          
          MeasurementPackage meas_package_R;
          istringstream iss_R(radar_measurment);
    	  long long timestamp_R;

    	  // reads first element from the current line
    	  string sensor_type_R;
    	  iss_R >> sensor_type_R;

      	  // read measurements at this timestamp
      	  meas_package_R.sensor_type_ = MeasurementPackage::RADAR;
          meas_package_R.raw_measurements_ = VectorXd(3);
          float ro;
      	  float theta;
      	  float ro_dot;
          iss_R >> ro;
          iss_R >> theta;
          iss_R >> ro_dot;
          meas_package_R.raw_measurements_ << ro,theta, ro_dot;
          iss_R >> timestamp_R;
          meas_package_R.timestamp_ = timestamp_R;
          
    	  ukf.ProcessMeasurement(meas_package_R);

    	  //target_x = ukf.x_[0];
    	  //target_y = ukf.x_[1];
    	  //Idea 1
    	  //simply make the car go in opposite x dir, shall work only for circular motion
    	  // if runaway car is going in straight line in x dir, this will fail!!
    	  //target_x = -ukf.x_[0];
    	  //target_y = ukf.x_[1];
    	  //Success in 4.8 sec

    	  //Idea2 add an offset to prediction based on velocity,yaw and yawrate,
    	  // only on x
    	  //target_x = ukf.x_[0] + ukf.x_[2]*cos(ukf.x_[3]) + ukf.x_[4];
    	  //target_y = ukf.x_[1];
    	  //Success in 5.1 sec

    	  //Idea3 add an offset to prediction based on velocity,yaw and yawrate,
    	  // only on y
    	  //target_x = ukf.x_[0] ;
    	  //target_y = ukf.x_[1] + ukf.x_[2]*sin(ukf.x_[3]) + ukf.x_[4];
    	  //Success in 7.5 sec

    	  //target_x = ukf.x_[0] + ukf.x_[2]*cos(ukf.x_[3]) + ukf.x_[4] ;
    	  //target_y = ukf.x_[1] + ukf.x_[2]*sin(ukf.x_[3]) + ukf.x_[4];
    	  //no success car goes out of map

    	  //Idea4 -- take running avg of last 5 prediction errors and add an offset based on
    	  //this avg. difference from measure can also be used but averaging will smoothen
    	  // the predictions
    	  //calculate avg
    	  float distance_difference_avg=0;
    	  float heading_difference_avg=0;
    	  for (int i=0;i<5;i++) {
    		  distance_difference_avg += heading_dist_diff[i];
    		  heading_difference_avg  += heading_dir_diff[i];
    	  }
    	  distance_difference_avg /=5;
    	  heading_difference_avg /=5;

    	  //overshoots x direction hence takes longer to come back to catchup
    	  //target_x = ukf.x_[0]+distance_difference_avg*cos(heading_difference_avg);
    	  //target_y = ukf.x_[1]+distance_difference_avg*sin(heading_difference_avg);
    	  //success at 9.6 s


    	  //less x more y offset,circular motion of car helps here
    	  target_x = ukf.x_[0]-distance_difference_avg*cos(heading_difference_avg);
    	  target_y = ukf.x_[1]+distance_difference_avg*sin(heading_difference_avg);
    	  //success at 4.56 s


    	  //less y more x offset,circular motion of car helps here
    	  //target_x = ukf.x_[0]+distance_difference_avg*cos(heading_difference_avg);
    	  //target_y = ukf.x_[1]-distance_difference_avg*sin(heading_difference_avg);
    	  //success at 9.5 s

    	  //if heading direction diff is > 22.5 degree and
    	  /*if (heading_difference_avg < M_PI/16) {
    		  target_x = ukf.x_[0]-distance_difference_avg*cos(heading_difference_avg);
    		  target_y = ukf.x_[1]+distance_difference_avg*sin(heading_difference_avg);
    	  } else if (heading_difference_avg > M_PI/16) {
    		  target_x = ukf.x_[0]+distance_difference_avg*cos(heading_difference_avg);
    		  target_y = ukf.x_[1]-distance_difference_avg*sin(heading_difference_avg);
    	  }

    	  //succes at 4.56 s */
    	  //if  distance_difference_avg diff is > 0.1  and
    	  /*if (heading_difference_avg < 1 ) {
    		  target_x = ukf.x_[0]-distance_difference_avg*cos(heading_difference_avg);
    		  target_y = ukf.x_[1]+distance_difference_avg*sin(heading_difference_avg);
    	  } else if (heading_difference_avg >= 1) {
    		  target_x = ukf.x_[0]+distance_difference_avg*cos(heading_difference_avg);
    		  target_y = ukf.x_[1]+distance_difference_avg*sin(heading_difference_avg);
    	  }

    	  //success at 4.56 s */

    	  //Idea 5, for intial steps (upto 10) add a high offset (1.5) to x/y pred
    	  //then after 10th pred roll back offset to 0.2
    	 //if starting distance diff is close the hunter will overshoot!
    	  /*
    	  if (distance_difference_avg > 1) {
    		  target_x = ukf.x_[0] + heading_dist_diff[4]*cos(heading_dir_diff[4]);
    		  target_y = ukf.x_[1] + heading_dist_diff[4]*cos(heading_dir_diff[4]);
    	  } else {
    		  target_x = ukf.x_[0] + 0.5;
    		  target_y = ukf.x_[1] + 0.5;
    	  } */

    	  //Idea 6, why not build another kalman filter to track the target car
    	  //state variables x,y shall do it (max velocity of target==hunter)
    	  //and keep motion model linear





    	  double heading_to_target = atan2(target_y - hunter_y, target_x - hunter_x);
    	  while (heading_to_target > M_PI) heading_to_target-=2.*M_PI; 
    	  while (heading_to_target <-M_PI) heading_to_target+=2.*M_PI;
    	  //turn towards the target
    	  double heading_difference = heading_to_target - hunter_heading;
    	  while (heading_difference > M_PI) heading_difference-=2.*M_PI; 
    	  while (heading_difference <-M_PI) heading_difference+=2.*M_PI;

    	  double distance_difference = sqrt((target_y - hunter_y)*(target_y - hunter_y) + (target_x - hunter_x)*(target_x - hunter_x));

    	  //add heading dir and distance difference
    	  //&heading_dir_diff,&heading_dist_diff,&mm
    	  heading_dir_diff[mm%5]=heading_difference;
    	  heading_dist_diff[mm%5]=distance_difference;
    	  ++mm;


          json msgJson;
          msgJson["turn"] = heading_difference;
          msgJson["dist"] = distance_difference; 
          auto msg = "42[\"move_hunter\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;

          //push the data into output file
          //out_file << p_x <<"\t" << p_y <<"\t"<< v1 <<"\t"<<v2 <<"\t"<< px_meas <<"\t" << py_meas<<"\t"<<x_gt<<"\t"<<y_gt<<"\t"<<vx_gt<<"\t"<<vy_gt<<"\t"<<RMSE(0)<<"\t"<<RMSE(1)<<"\t"<<RMSE(2)<<"\t"<<RMSE(3)<<"\t"<<ukf.NIS_<<"\t"<<ukf.NIS_laser_<<"\t"<<ukf.NIS_radar_<<endl;
          //#my_cols=['p1est','p2est','vest','yawest','yawrateest','p1meas','p2meas','p1','p2','v','yaw', 'yawrate','v1_gt','v2_gt', 'NIS_laser', 'NIS_radar']
          //out_file << p_x <<"\t" << p_y <<"\t"<< v <<"\t"<<yaw <<"\t"<< yawrate <<"\t"<< px_meas <<"\t"<< py_meas<<"\t"<<x_gt<<"\t"<<y_gt<<"\t"<<sqrt(vx_gt*vx_gt+vy_gt*vy_gt)<<"\t"<<yaw_gt<<"\t"<<yawrate_gt<<"\t"<<vx_gt<<"\t"<<vy_gt<<"\t"<<ukf.NIS_laser_<<"\t"<<ukf.NIS_radar_<<"\t"<<ukf.NIS_<<"\t"<<RMSE(0)<<"\t"<<RMSE(1)<<"\t"<<RMSE(2)<<"\t"<<RMSE(3)<<endl;

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
  //close the data capture file
  out_file.close();
}























































































