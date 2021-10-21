

#include "qingzhou_control.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "qingzhou_control");
  // ros::NodeHandle nh;
  // ros::NodeHandle private_nh("~");
  Actuator node;
  node.run();
  return 0;
}
