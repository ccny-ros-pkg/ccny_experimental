#include "KF/kf.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "KF");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  KF ekf(nh, nh_private);
  ros::spin();
  return 0;
}
