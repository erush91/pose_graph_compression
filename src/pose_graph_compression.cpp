#include <pose_graph_compression.h>

using namespace std;
using namespace Eigen;

pose_graph_compression::pose_graph_compression(ros::NodeHandle* nh)
{
  nh_ = nh;
  pub = nh_->advertise<std_msgs::Int16MultiArray>("/D01/pose_graph/compressed", 10);
  pub2 = nh_->advertise<nav_msgs::Path>("/D01/pose_graph/uncompressed", 10);
  pub_world = nh_->advertise<nav_msgs::Path>("path_world", 10);
  sub = nh_->subscribe<nav_msgs::Path>("/D01/lio_sam/mapping/path", 1, &pose_graph_compression::callback, this);
  sub2 = nh_->subscribe<std_msgs::Int16MultiArray>("/D01/pose_graph/compressed", 1, &pose_graph_compression::callback2, this);

  setup_timer();
}


void pose_graph_compression::setup_timer()
{
  ROS_INFO("%s: Creating timers ...", nh_->getNamespace().c_str());
  timerReplan_ = nh_->createTimer(ros::Duration(1.0), &pose_graph_compression::timer_cb, this);
}


void pose_graph_compression::timer_cb(const ros::TimerEvent&) // running at a fast rate
{
  if(init_flag)
  {
    cout << "-------------------------------------" << endl;
    std_msgs::Int16MultiArray arr;
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = path_in->header.frame_id;
    geometry_msgs::PoseStamped pose;
    float x, y, z;
    float x_last, y_last, z_last;
    float xyz_distance;
    int x_short_int, y_short_int, z_short_int;
    int i = 0;
    for(vector<geometry_msgs::PoseStamped>::const_iterator it = path_in->poses.begin(); it != path_in->poses.end(); it++)
    {
      x = path_in->poses[i].pose.position.x;
      y = path_in->poses[i].pose.position.y;
      z = path_in->poses[i].pose.position.z;
      xyz_distance  = (x - x_last) * (x - x_last)
                    + (y - y_last) * (y - y_last)
                    + (z - z_last) * (z - z_last);
      if(xyz_distance > 25)
      {
        path.poses.push_back(*it);
        x_last = x;
        y_last = y;
        z_last = z;
      }
      i++;
    }
    cout << "ORIGINAL FLOAT:        [" << x << ", " << y << ", " << z << "]" << endl;
    cout << "LAST:                  [" << x_last << ", " << y_last << ", " << z_last << "]" << endl;

    nav_msgs::Path pathMsgOut;
    pathMsgOut.header.frame_id = "world";
    pathMsgOut.header.stamp = ros::Time::now();
    pathMsgOut.poses.resize(path.poses.size());

    if(pathMsgOut.poses.size() < 1)
    {
      pub_world.publish(pathMsgOut);
      return;
    }

    geometry_msgs::TransformStamped pathInToOut;
    try
    {
      pathInToOut = tfBuffer.lookupTransform(pathMsgOut.header.frame_id, path.header.frame_id, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      return;
    }

    for(int i=0; i<pathMsgOut.poses.size(); i++)
      tf2::doTransform (path.poses[i].pose, pathMsgOut.poses[i].pose, pathInToOut);
    
    pub_world.publish(pathMsgOut);


    int j = 0;
    for(vector<geometry_msgs::PoseStamped>::const_iterator it = pathMsgOut.poses.begin(); it != pathMsgOut.poses.end(); it++)
    {
      x = pathMsgOut.poses[j].pose.position.x;
      y = pathMsgOut.poses[j].pose.position.y;
      z = pathMsgOut.poses[j].pose.position.z;

      x_short_int = static_cast<short int>(20 * x);
      y_short_int = static_cast<short int>(20 * y);
      z_short_int = static_cast<short int>(20 * z);

      arr.data.push_back(x_short_int);
      arr.data.push_back(y_short_int);
      arr.data.push_back(z_short_int);
      j++;
    }
    cout << "COMPRESSED SHORT INT:  [" << x_short_int << ", " << y_short_int << ", " << z_short_int << "]" << endl;
    pub.publish(arr);
  }
}


void pose_graph_compression::callback(const nav_msgs::Path::ConstPtr& msg)
{
  path_in = msg;
  init_flag = 1;
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
