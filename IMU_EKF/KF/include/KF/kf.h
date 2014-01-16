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

const double kGravity= 9.81;
const double kAngularVelocityThreshold = 0.2;
const double kAccelerationThreshold = 0.1;
const double kDeltaAngularVelocityThreshold = 0.01;
const double alpha_ = 0.01;
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
    
    ros::Publisher q0_publisher_;    
    ros::Publisher q1_publisher_;
    ros::Publisher q2_publisher_;
    ros::Publisher q3_publisher_;
    
    ros::Publisher q0_pred_publisher_;
    ros::Publisher q1_pred_publisher_;
    ros::Publisher q2_pred_publisher_;
    ros::Publisher q3_pred_publisher_;

    ros::Publisher q0_meas_publisher_;
    ros::Publisher q1_meas_publisher_;
    ros::Publisher q2_meas_publisher_;
    ros::Publisher q3_meas_publisher_;

    tf::TransformBroadcaster tf_broadcaster_;
    
    std::string fixed_frame_;
    std::string imu_frame_;
    std_msgs::Header imu_header_;
        
    double bias_alpha_;
    bool use_mag_;
    // **** state variables
  
    ros::Time time_prev_;
    bool initialized_filter_;
    boost::mutex mutex_;

    // TODO (idryanov) these are only needed for debug.    
    double mx_, my_, mz_, ax_, ay_, az_;

    double wx_prev_, wy_prev_, wz_prev_;
    double q0_prev_, q1_prev_, q2_prev_, q3_prev_;

    double bias_wx_, bias_wy_, bias_wz_;
    double q0_, q1_, q2_, q3_;  // quaternion
    double constant_dt_;

    // **** member functions
    void initializeParams();
  
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mav_msg);
    
    void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);

    void updateBiases(double ax, double ay, double az, 
                      double wx, double wy, double wz);

    void prediction(double wx, double wy, double wz, double dt,
                    double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred);

    void getOrientation(double ax, double ay, double az, 
                        double mx, double my, double mz, 
                        double& q0, double& q1, double& q2, double& q3);

    void getOrientation(double ax, double ay, double az, 
                        double q0_pred, double q1_pred, double q2_pred, double q3_pred, 
                        double& q0, double& q1, double& q2, double& q3);

  
    void quaternionMultiplication(double p0, double p1, double p2, double p3,
                                  double q0, double q1, double q2, double q3, 
                                  double& r0, double& r1, double& r2, double& r3);
    void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3);
    void invertQuaternion(
        double q0, double q1, double q2, double q3,
        double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv);

    void normalizeVector(double& x, double& y, double& z);

    tf::Quaternion hamiltonToTFQuaternion(
        double q0, double q1, double q2, double q3);
    void publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    void publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);

    double computeDeltaQuaternion(double q0, double q1, double q2, double q3);
    //void checkSolutions(double ax, double ay, double az, double& q1_acc,
    //double& q2_acc, double& q3_acc, double& q4_acc);
};

#endif // KF_H
