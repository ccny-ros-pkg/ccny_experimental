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
#include <KF/EXTERNAL/eigen3/Eigen/Eigen>
#include <KF/EXTERNAL/eigen3/Eigen/Geometry>
#include <KF/EXTERNAL/eigen3/Eigen/LU>
class KF
{
    typedef sensor_msgs::Imu              ImuMsg;
    typedef geometry_msgs::Vector3Stamped MagMsg;

    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber; 
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;
    typedef Eigen::Matrix<float, 4, 1> Vector4f;
  public:
    KF(ros::NodeHandle nh, ros::NodeHandle nh_private);    
    virtual ~KF();

  private:
      
    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
     boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<MagSubscriber> mag_subscriber_;

    ros::Subscriber pose_subscriber_;
    
    ros::Publisher pos_publisher_;    
    ros::Publisher acc_publisher_;
    ros::Publisher vel_publisher_;
    //ros::Subscriber imu_subscriber_;
    ros::Publisher unf_pos_publisher_;  
    ros::Publisher unf_vel_publisher_;
    ros::Publisher unf_acc_publisher_;
    ros::Publisher lin_acc_diff_publisher_;
    
    ros::Publisher imu_EKF_publisher_;
    
    ros::Publisher roll_kf_publisher_; 
    ros::Publisher pitch_kf_publisher_;
    ros::Publisher yaw_kf_publisher_;
    
    ros::Publisher roll_acc_publisher_;
    ros::Publisher pitch_acc_publisher_;
    tf::TransformBroadcaster tf_broadcaster_;

    Eigen::Vector3f gravity_vec_;
    Eigen::Vector3f h_ref_vec_;
    // **** paramaters
    Eigen::Matrix<float, 4, 4> P_;  // state covariance
    Eigen::Matrix<float, 4, 1> X_;  // State   
    Eigen::Quaternionf q_;    
    double sigma_g_;
    static const double gmag_= 9.81;
    double sigma_a_;
    double sigma_m_;
    float mx_prev_, my_prev_, mz_prev_;

    std::string fixed_frame_;
    std::string imu_frame_;
    std_msgs::Header imu_header_;
    
    tf::Transform f2b_prev_;
    tf::Vector3 imu_lin_acc_;
    tf::Vector3 lin_acc_diff_filtered_;
    tf::Vector3 lin_vel_;
    tf::Vector3 lin_acc_;
    geometry_msgs::Vector3 lin_acc_niac_;
    geometry_msgs::Vector3 lin_acc_vector_;
    std::vector<geometry_msgs::Vector3> ang_vel_;
    std::vector<geometry_msgs::Vector3> imu_acc_;
    std::vector<ros::Time> t_;

    tf::Vector3 pos_;
    geometry_msgs::Vector3 lin_acc_diff_vector_;
    geometry_msgs::PoseStamped::ConstPtr pose_msg_;
    // **** state variables
    ros::Time sync_time_;
    ros::Duration delay_;
    ros::Time timestamp_;
    bool initialized_;
    bool initialized_filter_;
    bool initialized_LP_filter_;
    ros::Time last_time_;
    ros::Time last_time_filter_;
    ros::Time last_time_imu_;
    boost::mutex mutex_;
    double latest_xpos_, latest_ypos_, latest_zpos_;
    double latest_xvel_, latest_yvel_, latest_zvel_;
    //std::vector<geometry_msgs::Vector3> ang_vel_;
    int imu_count_;
    bool vo_data_;

    double q0, q1, q2, q3;  // quaternion
    float b_ax, b_ay, b_az; // acceleration bias
    double constant_dt_;
    // **** member functions
    void initializeParams();
        
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw, const MagMsg::ConstPtr& mag_msg);
        
    void getPrediction(float wx, float wy, float wz, float dt, Vector4f& q_pred);
    void getMeasurement(
     float ax, float ay, float az, 
     float mx, float my, float mz, 
     float& lx, float& ly,    
     Vector4f& q_acc,
     Vector4f& q_mag);
    
     
     void getMeasCovariance(
     float ax, float ay, float az, 
     float mx, float my, float mz,  
     float lx, float ly,  
     Vector4f q_acc,
     Vector4f q_mag,
     Eigen::Matrix<float, 4, 4>& Q);

    void update(float ax, float ay, float az,
                float mx, float my, float mz, 
                const Vector4f q_pred);
    void quaternionMultiplication(Vector4f q_acc, Vector4f q_mag, Vector4f& q_meas);
   void makeContinuos(const Vector4f q_pred, Vector4f& q_meas);
    void publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
   
};

#endif // NIAC_H
