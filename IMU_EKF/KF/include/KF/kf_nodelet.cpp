#include "KF/kf_nodelet.h"

PLUGINLIB_DECLARE_CLASS(KF, KFNodelet, KFNodelet, nodelet::Nodelet);

void KFNodelet::onInit()
{
  NODELET_INFO("Initializing KF Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  kf_ = new KF(nh, nh_private);
}
