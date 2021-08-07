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
  float x, y, z;
  float x_last, y_last, z_last;
  float xyz_distance;
  int x_short_int, y_short_int, z_short_int;
  int i = 0;
  for(vector<geometry_msgs::PoseStamped>::const_iterator it = msg->poses.begin(); it != msg->poses.end(); it++)
  {
    x = msg->poses[i].pose.position.x;
    y = msg->poses[i].pose.position.y;
    z = msg->poses[i].pose.position.z;
    xyz_distance  = (x - x_last) * (x - x_last)
                  + (y - y_last) * (y - y_last)
                  + (z - z_last) * (z - z_last);
    if(xyz_distance > 25)
    {
      x_short_int = static_cast<short int>(20 * x);
      y_short_int = static_cast<short int>(20 * y);
      z_short_int = static_cast<short int>(20 * z);

      arr.data.push_back(x_short_int);
      arr.data.push_back(y_short_int);
      arr.data.push_back(z_short_int);
      x_last = x;
      y_last = y;
      z_last = z;
    }
    
    i++;
  }
  cout << "ORIGINAL FLOAT:        [" << x << ", " << y << ", " << z << "]" << endl;
  cout << "COMPRESSED SHORT INT:  [" << x_short_int << ", " << y_short_int << ", " << z_short_int << "]" << endl;
  cout << "LAST                :  [" << x_last << ", " << y_last << ", " << z_last << "]" << endl;
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
