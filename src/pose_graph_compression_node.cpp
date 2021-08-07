#include "pose_graph_compression.h"

using namespace std;

int main(int argc, char** argv)
{
        ros::init(argc, argv, "pose_graph_compression_node");
        ros::NodeHandle nh(ros::this_node::getName());

        pose_graph_compression poseGraphCompression(&nh);

        ros::spin();

        return 0;
}
