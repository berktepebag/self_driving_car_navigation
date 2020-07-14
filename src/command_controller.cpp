#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <rc_car_2dnav/CommandsConfig.h>

void callback(rc_car_2dnav::CommandsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d %d", 
            config.forward, config.backward, 
            config.steer);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamic_param_cmd");

  dynamic_reconfigure::Server<rc_car_2dnav::CommandsConfig> server;
  dynamic_reconfigure::Server<rc_car_2dnav::CommandsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}