#ifndef EKF_H
#define EKF_H

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
#include <EKF/EXTERNAL/eigen3/Eigen/Eigen>
#include <EKF/EXTERNAL/eigen3/Eigen/LU>
class EKF
{

  public:
    EKF(ros::NodeHandle nh, ros::NodeHandle nh_private);    
    virtual ~EKF();

  private:
    typedef sensor_msgs::Imu              ImuMsg;
    typedef geometry_msgs::Vector3Stamped MagMsg;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::Vector3Stamped> MySyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;    
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber; 
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

   // message_filters::Synchronizer<MySyncPolicy> * sync_;
   // message_filters::Subscriber<sensor_msgs::Imu> * imu_sub_;
   // message_filters::Subscriber<geometry_msgs::Vector3Stamped> * lin_acc_sub_;
    //message_filters::TimeSynchronizer<sensor_msgs::Imu, geometry_msgs::Vector3Stamped> *sync_;
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
    
    ros::Publisher roll_ekf_publisher_; 
    ros::Publisher pitch_ekf_publisher_;
    ros::Publisher yaw_ekf_publisher_;
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

    Eigen::Vector3f gravity_vec_;
    // **** paramaters
    Eigen::Matrix<float, 7, 7> P;  // state covariance
    Eigen::Matrix<float, 7, 1> X;  // State   
    double sigma_g_;
    
    double sigma_a_;
    double alpha_;
    double sigma_bax_;
    double sigma_bay_;
    double sigma_baz_;
    double sigma_bgx_;
    double sigma_bgy_;
    double sigma_bgz_;
    double sigma_h_;
    
    double hx_;
    double hy_;
    double hz_;
    
    float mx_, my_, mz_, ax_, ay_, az_;
    
    double threshold_a_;
    double threshold_g_;
    float yaw_g;
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

    double q0, q1, q2, q3;  // quaternion
    float b_ax, b_ay, b_az; // acceleration bias
    float b_gx_, b_gy_, b_gz_; // gyroscope bias
    float p_prev_, q_prev_, r_prev_, ax_prev_, ay_prev_, az_prev_ ;     
    double constant_dt_;
    // **** member functions

    void initializeParams();
    void callback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw, const  geometry_msgs::Vector3Stamped::ConstPtr lin_acc_msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mav_msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr pose_msg);
    void abgFilter(tf::Vector3 pos_reading, std_msgs::Header header, double dt);
    void filter(
      float p, float q, float r,
      float ax, float ay, float az,
      float mx, float my, float mz,
      float dt);
    void publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    void publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    // Fast inverse square-root
        // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
        static float invSqrt(float x)
        {
          float halfx = 0.5f * x;
          float y = x;
          long i = *(long*)&y;
          i = 0x5f3759df - (i>>1);
          y = *(float*)&i;
          y = y * (1.5f - (halfx * y * y));
          return y;
        }
};

#endif // NIAC_H
