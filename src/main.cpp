#include <actionlib/client/simple_action_client.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Twist.h>  //for geometry_msgs::Twist
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <stdlib.h>  
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// global variable
std::vector<std::vector<double>> targets;
int current_target_id{0};
geometry_msgs::Twist explorer_orientation;
nav_msgs::Odometry explorer_odom;

bool explorer_goal_sent = false;
bool follower_goal_sent = false;


void broadcast() {
  //for broadcaster
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  //broadcast the new frame to /tf Topic
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
  transformStamped.child_frame_id = "my_frame";

  transformStamped.transform.translation.x = 0.5;
  transformStamped.transform.translation.y = 0.5;
  transformStamped.transform.translation.z = 0.2;
  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 1;
  ROS_INFO("Broadcasting");
  br.sendTransform(transformStamped);
}

void listen(tf2_ros::Buffer& tfBuffer) {
  //for listener

  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform("map", "my_frame", ros::Time(0));
    auto trans_x = transformStamped.transform.translation.x;
    auto trans_y = transformStamped.transform.translation.y;
    auto trans_z = transformStamped.transform.translation.z;

    ROS_INFO_STREAM("Position in map frame: ["
      << trans_x << ","
      << trans_y << ","
      << trans_z << "]"
    );
  }
  catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}
void update_odom_value(nav_msgs::Odometry odom){
  explorer_odom = odom;
}

// robot rotate
void rotate_robot(ros::Publisher &cmd_publisher){
  
  std::cout << "Rotating the robot" << '\n';

  geometry_msgs::Twist explorer_orientation;
  explorer_orientation.angular.z = 0.8;
  cmd_publisher.publish(explorer_orientation);  
  std::cout << explorer_odom.pose.pose.orientation.z << '\n';
  while (true)
  {
    std::cout << explorer_odom.pose.pose.orientation.z << '\n';
  }
  
  // std::cout << std::abs(explorer_odom.pose.pose.orientation.z) << '\n';
  // while(std::abs(explorer_odom.pose.pose.orientation.z) > 0.001){};
}

// build goal for explorer
move_base_msgs::MoveBaseGoal fetch_next_explorer_goal(){
  
  move_base_msgs::MoveBaseGoal explorer_goal;
  std::vector<double> next_target{targets[current_target_id]};

  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = next_target[0];
  explorer_goal.target_pose.pose.position.y = next_target[1];
  explorer_goal.target_pose.pose.orientation.w = 1.0;

  return explorer_goal;
}

int main(int argc, char** argv)
{
  

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;

  // publishers
  ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 2);
  ros::Subscriber odom_subscriber = nh.subscribe<nav_msgs::Odometry>("/explorer/odom", 1, update_odom_value);

  // fetch all the targets from the parameter server
  for(int i=1; i<5; i++){
    std::string target_topic{"/aruco_lookup_locations/target_" + std::to_string(i)};
    std::vector<double> target;    
    nh.getParam(target_topic, target);
    targets.push_back(target);
  }  

  // exit (EXIT_FAILURE);

  // tell the action client that we want to spin a thread by default
  MoveBaseClient explorer_client("/explorer/move_base", true);

  // // tell the action client that we want to spin a thread by default
  // MoveBaseClient follower_client("/follower/move_base", true);

  // wait for the action server to come up
  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  // while (!follower_client.waitForServer(ros::Duration(5.0))) {
  //   ROS_INFO("Waiting for the move_base action server to come up for follower");
  // }

  
  // move_base_msgs::MoveBaseGoal follower_goal;

  

  // //Build goal for follower
  // follower_goal.target_pose.header.frame_id = "map";
  // follower_goal.target_pose.header.stamp = ros::Time::now();
  // follower_goal.target_pose.pose.position.x = -0.289296;//
  // follower_goal.target_pose.pose.position.y = -1.282680;//
  // follower_goal.target_pose.pose.orientation.w = 1.0;


  // // explorer_client.waitForResult();

  // // ROS_INFO("Sending goal");
  // // follower_client.sendGoal(follower_goal);
  // // explorer_client.waitForResult();


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ROS_INFO("YOUR TARGET IS PRINTED");
    // ROS_INFO("Target 1 ([%f], [%f])", target_1[0]);
    if (!explorer_goal_sent) {
      ROS_INFO("Sending goal for explorer");
      move_base_msgs::MoveBaseGoal goal = fetch_next_explorer_goal();
      explorer_client.sendGoal(goal);//this should be sent only once
      explorer_goal_sent = true;
    }
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Robot reached goal [%d]", current_target_id);

      rotate_robot(cmd_publisher);
      if(current_target_id < 5){
        current_target_id ++;
        explorer_goal_sent = false;
      }
      
    }
    // if (!follower_goal_sent) {
    //   ROS_INFO("Sending goal for explorer");
    //   follower_client.sendGoal(follower_goal);//this should be sent only once
    //   follower_goal_sent = true;
    // }
    // if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //   ROS_INFO("Hooray, robot reached goal");
    // }

    broadcast();
    listen(tfBuffer);
    //ros::spinOnce(); //uncomment this if you have subscribers in your code
    loop_rate.sleep();
  }


}