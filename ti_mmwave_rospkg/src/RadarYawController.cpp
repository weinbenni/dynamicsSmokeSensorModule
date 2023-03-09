#include "ros/ros.h"
#include "ti_mmwave_rospkg/RadarScan.h"
#include "ti_mmwave_rospkg/RadarCube.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <string>
#include <tf/transform_broadcaster.h>
#include <signal.h>
// #include "NumCpp.hpp"
#include <math.h>

#define STEP_INCREMENT 2.5
#define MAX_ANGLE ((295 - 45) * M_PI / 180)
#define MIN_ANGLE ((115 + 45) * M_PI / 180)
#define MAX_ANGLE_DEG MAX_ANGLE * 180 / M_PI
#define MIN_ANGLE_DEG MIN_ANGLE * 180 / M_PI
#define START_ANGLE ((180 + 15) * M_PI / 180)
#define START_ANGLE_DEG START_ANGLE * 180 / M_PI
// #define STEPS_PER_HALF_REV 6
// #define STEP_INCREMENT 180 / STEPS_PER_HALF_REV

float min_angle = MIN_ANGLE;
float max_angle = MAX_ANGLE;

// ros::Publisher radar_yaw_position;
ros::Publisher radar_yaw_cmd;
float current_yaw = M_PI;
float current_yaw_deg = START_ANGLE_DEG;
int current_yaw_idx = 180 / STEP_INCREMENT / 2;
float current_yaw_dir = STEP_INCREMENT;
std_msgs::Float64 velocity;
int smooth_yaw;

// nc::NdArray<float> positions = nc::arange<float>(90, 270, (int)(180 / STEP_INCREMENT) + 1);
#define POS_SIZE 36
// float positions[] = { 90., 105., 120., 135., 150., 165., 180., 195., 210., 225., 240., 255., 270. };
float positions[] = { 90.,  95., 100., 105., 110., 115., 120., 125., 130., 135., 140.,
       145., 150., 155., 160., 165., 170., 175., 180., 185., 190., 195.,
       200., 205., 210., 215., 220., 225., 230., 235., 240., 245., 250.,
       255., 260., 265., 270. };
// auto positions = nc::hstack( { nc::arange<float>(PI, PI + PI / 2 + 1, STEP_INCREMENT / 2), 
//                                nc::arange<float>(PI + PI / 2, PI - PI / 2, STEPS_PER_HALF_REV), 
//                                nc::arange<float>(PI - PI / 2, PI, STEPS_PER_HALF_REV / 2), });

void transform_yaw_pos(float current_yaw) {
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // // transform.setOrigin( tf::Vector3(-0.065, 0.000, 0.043) );  // TODO Update according to URDF!
  // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  // tf::Quaternion q;
  // q.setRPY(0, 0, current_yaw + (M_PI * -195 / 180));  // TODO variable Mounting position, not only 195°
  // // q.setRPY(0, 0, current_yaw);  // TODO
  // transform.setRotation(q);
  // // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "radar_link"));
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "radar_link", "ti_mmwave_pcl"));
}

void set_velocity(const sensor_msgs::JointState &MotorState) {
  radar_yaw_cmd.publish(velocity);
  // transform_yaw_pos(MotorState.position[0]);
  if (MotorState.position[0] > max_angle) {
    velocity.data = -abs(velocity.data);
    radar_yaw_cmd.publish(velocity);
  }
  if (MotorState.position[0] < min_angle) {
    velocity.data = abs(velocity.data);
    radar_yaw_cmd.publish(velocity);
  }
}

void transform_pcl(const sensor_msgs::JointState &MotorState) {
  transform_yaw_pos(MotorState.position[0]);
}

void set_position_deg(float new_yaw) {
  std_msgs::Float64 new_yaw_rad;
  new_yaw_rad.data = new_yaw * M_PI / 180;  // deg to rad
  ROS_INFO("Moving to new pos: %f", new_yaw);
  radar_yaw_cmd.publish(new_yaw_rad);
  current_yaw = new_yaw_rad.data;
  transform_yaw_pos(current_yaw);
}

void set_next_pos() {
  current_yaw_deg += current_yaw_dir;
  if (current_yaw_deg > MAX_ANGLE_DEG) {
    current_yaw_dir = -abs(current_yaw_dir);
    current_yaw_deg += 2*current_yaw_dir;
  }
  else if (current_yaw_deg < MIN_ANGLE_DEG) {
    current_yaw_dir = abs(current_yaw_dir);
    current_yaw_deg += 2*current_yaw_dir;
  }
  set_position_deg(current_yaw_deg);
  // current_yaw_idx += current_yaw_dir;
  // if (current_yaw_idx > POS_SIZE) {
  //   current_yaw_idx = POS_SIZE - 1;
  //   current_yaw_dir = -1;
  // }
  // if (current_yaw_idx < 0) {
  //   current_yaw_idx = 1;
  //   current_yaw_dir = 1;
  // }
  // set_position_deg(positions[current_yaw_idx]);
}

void move_to_new_pos(const ti_mmwave_rospkg::RadarCube &RadarScan) {
  ros::Duration(0.1).sleep();  // wait for the radar to perform the chirps and go into transfer state.
  set_next_pos(); // move radar while in transfer state
}

void stop_motor(int sig) {
  if (smooth_yaw) {
    velocity.data = 0;
    radar_yaw_cmd.publish(velocity);
    ros::shutdown();
  }
  else {
    set_position_deg(START_ANGLE_DEG);
    ros::shutdown();
  }
}



int main(int argc, char **argv)
{
  if (argc < 5) {
    ROS_INFO("Please provide enough arguments: ControllerTopic, Min_Angle, Max_Angle, Velocity");
    return 0;
  }
  ros::init(argc, argv, "radar_yaw_controller");
  ros::NodeHandle n;
  std::string controller_cmd_topic = argv[1];  // /arm_control/radar_yaw_position_controller/command
  // smooth_yaw = atoi(argv[2]);
  smooth_yaw = 1;
  min_angle = ((205 + (float)atoi(argv[2])) * M_PI / 180);
  max_angle = ((205 + (float)atoi(argv[3])) * M_PI / 180);
  float vel = atof(argv[4]);
  ROS_INFO("Min Angle / Max Angle: %.2f / %.2f", min_angle * 180 / M_PI, max_angle * 180 / M_PI);
  radar_yaw_cmd = n.advertise<std_msgs::Float64>(controller_cmd_topic, 10);
  ros::Subscriber sub;
  ros::Subscriber sub_rate;
  if (!smooth_yaw) {
    set_position_deg(START_ANGLE);
    // radar_yaw_cmd.publish(START_ANGLE);
    sub = n.subscribe("/ti_mmwave/radar_cube", 10, move_to_new_pos);
    // sub_rate = n.subscribe("/joint_states", 10, transform_pcl);
  }
  else {
    velocity.data = vel;
    // ROS_INFO("Smooth control, %f", velocity.data);
    radar_yaw_cmd.publish(velocity);
    sub = n.subscribe("/joint_states", 1, set_velocity);
  }
  signal(SIGINT, stop_motor);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}