#ifndef KF_KF_NODELET_H
#define KF_KF_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "KF/kf.h"

class KFNodelet : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
      KF * kf_;  // FIXME: change to smart pointer
};

#endif // KF_KF_NODELET_H
