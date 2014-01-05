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
#include <KF/EXTERNAL/eigen3/Eigen/LU>

const double kGravity= 9.81;
const double Deviation = 1.18;  
const double kAngularVelocityThreshold = 0.2;
const double kAccelerationThreshold = 0.3;
const double kDeltaAngularVelocityThreshold = 0.002;

class KF
{
  public:
    KF(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);    
    virtual ~KF();

  private:
    typedef sensor_msgs::Imu              ImuMsg;
    typedef geometry_msgs::Vector3Stamped MagMsg;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::Vector3Stamped> MySyncPolicy;
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

    ros::Subscriber pose_subscriber_;
    
    ros::Publisher pos_publisher_;    
    ros::Publisher acc_publisher_;
    ros::Publisher vel_publisher_;
    //ros::Subscriber imu_subscriber_;
    ros::Publisher unf_pos_publisher_;  
    ros::Publisher unf_vel_publisher_;
    ros::Publisher unf_acc_publisher_;
    ros::Publisher lin_acc_diff_publisher_;
    
    ros::Publisher imu_KF_publisher_;
    
    ros::Publisher roll_ekf_publisher_; 
    ros::Publisher pitch_ekf_publisher_;
    ros::Publisher yaw_ekf_publisher_;
    ros::Publisher roll_xekf_publisher_; 
    ros::Publisher pitch_xekf_publisher_;
    ros::Publisher yaw_xekf_publisher_;
    ros::Publisher yaw_g_publisher_;
    ros::Publisher yaw_m_publisher_;
    ros::Publisher bias_ax_publisher_;
    ros::Publisher bias_ay_publisher_;    
    ros::Publisher bias_az_publisher_;
    ros::Publisher bias_gx_publisher_;
    ros::Publisher bias_gy_publisher_;    
    ros::Publisher bias_gz_publisher_;
    ros::Publisher roll_acc_publisher_;
    ros::Publisher pitch_acc_publisher_;

    ros::Publisher axfil_publisher_;
    ros::Publisher bxfil_publisher_; 
   
    tf::TransformBroadcaster tf_broadcaster_;

    Eigen::Vector3d gravity_vec_;
    Eigen::Vector3d magnetic_vec_;
    // **** paramaters
    bool do_bias_estimation_;

    Eigen::Matrix<double, 4, 4> P_;  // state covariance
    Eigen::Matrix<double, 4, 1> X_;  // State   
    Eigen::Matrix3d Sigma_g_, Sigma_a_, Sigma_h_;
    double sigma_gx_;
    double sigma_gy_;
    double sigma_gz_;

    double h_norm_; 
    double sigma_ax_;
    double sigma_ay_;
    double sigma_az_;
    double alpha_;
    double sigma_bax_;
    double sigma_bay_;
    double sigma_baz_;
    double sigma_bgx_;
    double sigma_bgy_;
    double sigma_bgz_;
    double sigma_hx_;
    double sigma_hy_;
    double sigma_hz_;
    bool use_mag_;
    double hx_;
    double hy_;
    double hz_;
    
    double mx_, my_, mz_, ax_, ay_, az_;
    
    double threshold_a_;
    double threshold_g_;
    double yaw_g_;
    std::string fixed_frame_;
    std::string imu_frame_;
    std_msgs::Header imu_header_;
    double gain_;     // algorithm gain
    double gamma_;    // low pass filter gain
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

    double q1_, q2_, q3_, q4_;  // quaternion
    double b_ax, b_ay, b_az; // acceleration bias
    double b_gx_, b_gy_, b_gz_; // gyroscope bias
    double p_prev_, q_prev_, r_prev_, ax_prev_, ay_prev_, az_prev_ ;     
    double constant_dt_;

    // **** member functions
    void initializeParams();

    void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mav_msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr pose_msg);

    void updateBiases(double ax, double ay, double az, double p, double q, double r);
    void prediction(double p, double q, double r, double dt);
    void correctionWithMag(double ax, double ay, double az,
                           double mx, double my, double mz);
    void correctionNoMag(double ax, double ay, double az);
    double getInclination(double ax, double ay, double az, double mx, double my, double mz);
    void getReferenceField(double mx, double my, double mz);
    void publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    void publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);

    static Eigen::Vector3d rotateVectorByQuaternion(
        const Eigen::Vector3d& v,
        double q1, double q2, double q3, double q4);
};

#endif // KF_H
