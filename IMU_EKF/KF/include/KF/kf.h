#ifndef KF_H
#define KF_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <Eigen/LU>
#include <Eigen/Eigen>


class KF
{
  public:
    KF(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);    
    virtual ~KF();

  private:
    typedef sensor_msgs::Imu              ImuMsg;
    typedef geometry_msgs::Vector3Stamped MagMsg;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, 
        geometry_msgs::Vector3Stamped> MySyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;    
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber; 
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<MagSubscriber> mag_subscriber_;

    
    ros::Publisher imu_KF_publisher_;
    
    ros::Publisher roll_ekf_publisher_; 
    ros::Publisher pitch_ekf_publisher_;
    ros::Publisher yaw_ekf_publisher_;
    ros::Publisher roll_xekf_publisher_; 
    ros::Publisher pitch_xekf_publisher_;
    ros::Publisher yaw_xekf_publisher_;
    ros::Publisher yaw_g_publisher_;
    ros::Publisher yaw_m_publisher_;
    
    ros::Publisher roll_acc_publisher_;
    ros::Publisher pitch_acc_publisher_;

      
    tf::TransformBroadcaster tf_broadcaster_;
    
    double mx_, my_, mz_, ax_, ay_, az_;
    std::string fixed_frame_;
    std::string imu_frame_;
    std_msgs::Header imu_header_;
    
    
    std::vector<ros::Time> t_;

     // **** state variables
    ros::Time sync_time_;
    ros::Time timestamp_;
    bool initialized_;
    bool initialized_filter_;
    boost::mutex mutex_;
    
    double q1_, q2_, q3_, q4_;  // quaternion
    double constant_dt_;

    // **** member functions
    void initializeParams();
  
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mav_msg);
    void getOrientation(double ax, double ay, double az, double mx, double my,
       double mz, 
                         double& q1, double& q2, double& q3, double& q4);
    void normalizeQuaternion(double& q1, double& q2, double& q3, double& q4);
    void normalizeVector(double& x, double& y, double& z);
    void publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    void publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
 
};

#endif // KF_H
