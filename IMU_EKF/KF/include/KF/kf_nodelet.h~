#ifndef EKF_EKF_NODELET_H
#define EKF_EKF_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "EKF/ekf.h"

class EKFNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
      EKF * ekf_;  // FIXME: change to smart pointer
};

#endif // EKF_EKF_NODELET_H
