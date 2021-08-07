#include <pose_graph_compression.h>

using namespace std;
using namespace Eigen;

pose_graph_compression::pose_graph_compression(ros::NodeHandle* nh)
{
  pub = nh->advertise<std_msgs::Int16MultiArray>("/D01/pose_graph/compressed", 10);
  pub2 = nh->advertise<nav_msgs::Path>("/D01/pose_graph/uncompressed", 10);
  sub = nh->subscribe<nav_msgs::Path>("/D01/lio_sam/mapping/path", 1, &pose_graph_compression::callback, this);
  sub2 = nh->subscribe<std_msgs::Int16MultiArray>("/D01/pose_graph/compressed", 1, &pose_graph_compression::callback2, this);
}

void pose_graph_compression::callback(const nav_msgs::Path::ConstPtr& msg)
{
  cout << "-------------------------------------" << endl;
  std_msgs::Int16MultiArray arr;
  float x_float, y_float, z_float;
  int x_short_int, y_short_int, z_short_int;
  int i = 0;
  for(vector<geometry_msgs::PoseStamped>::const_iterator it = msg->poses.begin(); it != msg->poses.end(); it++)
  {
    x_float = msg->poses[i].pose.position.x;
    y_float = msg->poses[i].pose.position.y;
    z_float = msg->poses[i].pose.position.z;
    
    x_short_int = static_cast<short int>(20 * x_float);
    y_short_int = static_cast<short int>(20 * y_float);
    z_short_int = static_cast<short int>(20 * z_float);

    arr.data.push_back(x_short_int);
    arr.data.push_back(y_short_int);
    arr.data.push_back(z_short_int);

    i++;
  }
  cout << "ORIGINAL FLOAT:        [" << x_float << ", " << y_float << ", " << z_float << "]" << endl;
  cout << "COMPRESSED SHORT INT:  [" << x_short_int << ", " << y_short_int << ", " << z_short_int << "]" << endl;
  pub.publish(arr);
}

void pose_graph_compression::callback2(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  geometry_msgs::PoseStamped pose;
  int i = 0;
  for(std::vector<short int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it)
  {
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    if(i == 0)
    {
      pose.pose.position.x = 0.05 * static_cast<float>(*(it));
      i++;
    }
    else if(i == 1)
    {
      pose.pose.position.y = 0.05 * static_cast<float>(*(it));
      i++;
    }
    else
    {
      pose.pose.position.z = 0.05 * static_cast<float>(*(it));
      path.poses.push_back(pose);
      i = 0;
    }
  }
  cout << "UNCOMPRESSED FLOAT:    [" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << "]" << endl;
  pub2.publish(path);
}

pose_graph_compression::~pose_graph_compression()
{
}
