#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class pose_graph_compression
{
  
private:
  ros::NodeHandle* nh_;
  ros::Publisher pub;
  ros::Publisher pub2;
  ros::Publisher pub_world;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer);

  nav_msgs::Path path_in;

  ros::Timer timerReplan_;
  double timeIntReplan_;
  bool init_flag = 0;

public:
  pose_graph_compression(ros::NodeHandle*);
  ~pose_graph_compression();
  void setup_timer();
  void timer_cb(const ros::TimerEvent&);
  void callback(const nav_msgs::Path::ConstPtr& msg);
  void callback2(const std_msgs::Int16MultiArray::ConstPtr& msg);

};