#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "imu_complementary_filter/complementary_filter.h"

namespace imu_tools {

class ComplementaryFilterROS
{
  public:
    ComplementaryFilterROS(const ros::NodeHandle& nh, 
                           const ros::NodeHandle& nh_private);    
    virtual ~ComplementaryFilterROS();

  private:

    // Convenience typedefs
    typedef sensor_msgs::Imu ImuMsg;
    typedef geometry_msgs::Vector3Stamped MagMsg;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, 
        geometry_msgs::Vector3Stamped> MySyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> 
        SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;    
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber; 
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

    // ROS-related variables.
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<MagSubscriber> mag_subscriber_;

    ros::Publisher imu_publisher_;
    ros::Publisher roll_publisher_;
    ros::Publisher pitch_publisher_;
    ros::Publisher yaw_publisher_;
    ros::Publisher state_publisher_;
    tf::TransformBroadcaster tf_broadcaster_;
         
    // Parameters:
    bool use_mag_;
    double constant_dt_;
    std::string fixed_frame_;

    // State variables:
    ComplementaryFilter filter_;
    ros::Time time_prev_;
    bool initialized_filter_;

    void initializeParams();
    void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mav_msg);
    void publish(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);

    tf::Quaternion hamiltonToTFQuaternion(
        double q0, double q1, double q2, double q3) const;
};

}  // namespace imu_tools

#endif // IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
