#include <ros/ros.h>
#include <mavconn/interface.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavlink/v2.0/mavlink_helpers.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include "NE_utilities.h"

mavros_msgs::State current_state;
sensor_msgs::NavSatFix NavSatFixed;
float compass_hdg, GYM_OFFSET; //rad

uint8_t system_id = 255;
uint8_t component_id = 1;
uint8_t target_system = 1;

bool packMavlinkMessage(const mavlink::Message& mavMsg, mavros_msgs::Mavlink &rosMsg)
{
  mavlink::mavlink_message_t msg;
  mavlink::MsgMap map(msg);
  mavMsg.serialize(map);
  auto mi = mavMsg.get_message_info();

  mavlink::mavlink_status_t *status = mavlink::mavlink_get_channel_status(mavlink::MAVLINK_COMM_0);
  status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
  mavlink::mavlink_finalize_message_buffer(&msg, system_id, component_id, status, mi.min_length, mi.length, mi.crc_extra);

  return mavros_msgs::mavlink::convert(msg, rosMsg);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

//get current GPS position of drone
void GPS_cb(const sensor_msgs::NavSatFix::ConstPtr& NavSatFix_)
{
	NavSatFixed=*NavSatFix_;
}

//get current compass heading of drone
void compass_cb(const std_msgs::Float64ConstPtr& compass_hdg_)
{
	compass_hdg=deg2rad(compass_hdg_->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_home_node");
  ros::NodeHandle home_handle;

  ros::Subscriber state_sub = home_handle.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
  ros::ServiceClient home_set = home_handle.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home",1);
  ros::Publisher origin_pub = home_handle.advertise<mavros_msgs::Mavlink>("mavlink/to", 1000);
  ros::Subscriber gps_sub = home_handle.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, GPS_cb);
  ros::Subscriber compass_sub = home_handle.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 1, compass_cb);
  ros::Rate rate(20.0);

  while(ros::ok() && current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  ros::Duration(2.0).sleep();
  
  //if(NavSatFixed.status.status<=0){
  //  ROS_ERROR("No NavSatFix. Exiting....");
  //  return -1;
  //}

  double latitude = NavSatFixed.latitude;
  double longitude = NavSatFixed.longitude;
  double altitude = NavSatFixed.altitude;

  mavlink::common::msg::SET_GPS_GLOBAL_ORIGIN originMsg;
  originMsg.latitude = (uint32_t)(latitude * 10000000);
  originMsg.longitude = (uint32_t)(longitude * 10000000);
  originMsg.altitude = (uint32_t)(altitude * 1000);
  originMsg.target_system = target_system;

  mavros_msgs::Mavlink packedMsg;
  bool success = packMavlinkMessage(originMsg, packedMsg);

  if(success){
    ROS_INFO("Pack mavlink message SUCCEEDED\n");
  }
  else{
    ROS_INFO("Pack mavlink message FAILED\n");
  }

  origin_pub.publish(packedMsg);

  ros::spinOnce();
  rate.sleep();
  ros::Duration(2.0).sleep();
  
  //set the orientation of the gym
  //   GYM_OFFSET = 0;
  //   for (int i = 1; i <= 30; ++i) {
  //     ros::spinOnce(); 
  //     rate.sleep();
  //     GYM_OFFSET += compass_hdg;
  //     ROS_INFO("current heading%d: %f", i, GYM_OFFSET/i);
  //   }
  //   GYM_OFFSET /= 30;
  //   ROS_INFO("the N' axis offest is facing: %f", rad2deg(GYM_OFFSET));
  
  ///////////////////////NE UPDATE/////////////////////////////////////////
  mavlink::common::msg::SET_HOME_POSITION homePosMsg;
  homePosMsg.latitude = (uint32_t)(latitude * 10000000);
  homePosMsg.longitude = (uint32_t)(longitude * 10000000);
  homePosMsg.altitude = (uint32_t)(altitude * 1000);
  homePosMsg.target_system = target_system;
  
  homePosMsg.x = homePosMsg.y = homePosMsg.z = 0;   
  homePosMsg.approach_x = homePosMsg.approach_y = 0.0;
  homePosMsg.approach_z = 1.0;
  
  homePosMsg.q[0] = 1.0; // w x y z
  homePosMsg.q[1] = homePosMsg.q[2] = homePosMsg.q[3] = 0.0;
  
  mavros_msgs::Mavlink packedSetHomeMsg;
  if(packMavlinkMessage(homePosMsg, packedSetHomeMsg))	ROS_INFO("Pack mavlink SET_HOME message SUCCEEDED\n");
  else	ROS_INFO("Pack mavlink SET_HOME message FAILED\n");

  origin_pub.publish(packedSetHomeMsg);
  ros::spinOnce();
  rate.sleep();
  ros::Duration(2.0).sleep();
  ///////////////////////NE UPDATE END/////////////////////////////////////////
  
//   mavros_msgs::CommandHome set_home_req;
//   set_home_req.request.current_gps = false; //if set to true probably FCU will get current GPS coords for this
//   set_home_req.request.latitude = latitude;
//   set_home_req.request.longitude = longitude;
//   set_home_req.request.altitude = altitude;
  //From MavLink --> Yaw angle. NaN to use default heading! Caution here.
  //set_home_req.request.yaw = GYM_OFFSET;

  //ros::service::call("/mavros/cmd/set_home", set_home_req);
  //printf("Result was %d\n", set_home_req.response.result);

}
