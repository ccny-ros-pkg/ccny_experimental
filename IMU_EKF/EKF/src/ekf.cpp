
#include "EKF/ekf.h"
#include <EKF/EXTERNAL/eigen3/Eigen/Eigen>
#include <EKF/EXTERNAL/eigen3/Eigen/LU>
#include <iostream>

EKF::EKF(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  initialized_filter_(false),
  initialized_LP_filter_(false),
  q0(1.0), q1(0.0), q2(0.0), q3(0.0)
  
{
  ROS_INFO ("Starting EKF");

  // **** get paramters 
  // nothing for now
  
  initializeParams();
  
  int queue = 5;

  imu_EKF_publisher_ = nh_.advertise<sensor_msgs::Imu>(
	    "imu_EKF/data", queue);

  roll_ekf_publisher_    = nh.advertise<std_msgs::Float32>("roll_ekf", queue);
  pitch_ekf_publisher_   = nh.advertise<std_msgs::Float32>("pitch_ekf", queue);
  yaw_ekf_publisher_     = nh.advertise<std_msgs::Float32>("yaw_ekf", queue);
  bias_ax_publisher_     = nh.advertise<std_msgs::Float32>("bias_ax", queue);
  bias_ay_publisher_     = nh.advertise<std_msgs::Float32>("bias_ay", queue);
  bias_az_publisher_     = nh.advertise<std_msgs::Float32>("bias_az", queue);
  roll_acc_publisher_    = nh.advertise<std_msgs::Float32>("roll_acc", queue);
  pitch_acc_publisher_   = nh.advertise<std_msgs::Float32>("pitch_acc", queue);
  //acc_mag_publisher_ = nh.advertise<std_msgs::Float32>("acc_mag", queue);
  //g_mag_publisher_   = nh.advertise<std_msgs::Float32>("g_mag", queue);

  // **** register subscribers
  int queue_size = 5;

  //imu_subscriber_ = nh_.subscribe(
  //  "imu/data_raw", queue_size, &EKF::imuCallback, this);

   imu_subscriber_.reset(new ImuSubscriber(
    nh_, "imu/data_raw", queue_size));
  if (use_mag_)
  {
    mag_subscriber_.reset(new MagSubscriber(
      nh_, "imu/mag", queue_size));

    sync_.reset(new Synchronizer(
      SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
    sync_->registerCallback(boost::bind(&EKF::imuMagCallback, this, _1, _2));
  }
  else
  {
    imu_subscriber_->registerCallback(&EKF::imuCallback, this);
  }
}


EKF::~EKF()
{
  ROS_INFO ("Destroying EKF");
}

void EKF::initializeParams()
{
   //last_time_ = 0.0;
 // initialized_ = false;
  if (!nh_private_.getParam ("use_mag", use_mag_))
   use_mag_ = false;

  if (!nh_private_.getParam ("sigma_g", sigma_g_))
    sigma_g_ = 0.01;
  
  if (!nh_private_.getParam ("sigma_a", sigma_a_))
    sigma_a_ = 0.01;

  if (!nh_private_.getParam ("sigma_m", sigma_m_))
    sigma_m_ = 0.01;
  
  if (!nh_private_.getParam ("constant_dt", constant_dt_))
      constant_dt_ = 0.0;

  if (!nh_private_.getParam ("gain", gain_))
     gain_ = 0.1;

  if (!nh_private_.getParam ("sigma_bx", sigma_bx_))
    sigma_bx_ = 0.01;
  if (!nh_private_.getParam ("sigma_by", sigma_by_))
    sigma_by_ = 0.01;
  if (!nh_private_.getParam ("sigma_bz", sigma_bz_))
    sigma_bz_ = 0.01;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
     fixed_frame_ = "odom";
  if (!nh_private_.getParam ("imu_frame", imu_frame_))
       imu_frame_ = "imu_niac";
  if (!nh_private_.getParam ("threshold", threshold_))
       threshold_ = 10.0;
  
  gravity_vec_ = Eigen::Vector3f(0, 0, gmag_);
  h_ref_vec_  = Eigen::Vector3f(0.193, -0.046, 0.503);
  
}


void EKF::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{

  boost::mutex::scoped_lock(mutex_);
     
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 

  ros::Time time = imu_msg_raw->header.stamp;
  

  float a_x = lin_acc.x;
  float a_y = lin_acc.y;
  float a_z = lin_acc.z;

    
  float dt;
  if (!initialized_filter_)
    {
      
      // initialize roll/pitch orientation from acc. vector
  	  double roll  = atan2(a_y, sqrt(a_x*a_x + a_z*a_z));
  	  double pitch = atan2(-a_x, sqrt(a_y*a_y + a_z*a_z));
  	  double yaw = 0.0;

      tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);

      X_(1) = init_q.getX();
      X_(2) = init_q.getY();
      X_(3) = init_q.getZ();
      X_(0) = init_q.getW();
  
            
      P_ = Eigen::MatrixXf::Zero(4,4);
   
      // initialize time
      last_time_filter_ = time;
      initialized_filter_ = true;
    }
    else
     {
      // determine dt: either constant, or from IMU timestamp

      if (constant_dt_ > 0.0)
    	  dt = constant_dt_;
      else 
      	dt = (time - last_time_filter_).toSec();
     
      last_time_filter_ = time;
      
      getPrediction(ang_vel.x, ang_vel.y, ang_vel.z, dt);

      //filter(ang_vel.x, ang_vel.y, ang_vel.z,
      //lin_acc.x, lin_acc.y, lin_acc.z,
      //dt);
      update(lin_acc.x, lin_acc.y, lin_acc.z, dt);
     
      std_msgs::Float32 acc_mag_msg;
      publishFilteredMsg(imu_msg_raw);
       
     }
}

void EKF::imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                         const MagMsg::ConstPtr& mag_msg)   
{

  boost::mutex::scoped_lock(mutex_);

  //sensor_msgs::Imu imu =  *imu_msg_raw;
  
  //imu.header.stamp = ros::Time::now();
    
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& mag_fld = mag_msg->vector;
  ros::Time time = imu_msg_raw->header.stamp;
 
  float a_x = lin_acc.x;
  float a_y = lin_acc.y;
  float a_z = lin_acc.z;

  //float a_magnitude = sqrt(a_x*a_x + a_y*a_y + a_z*a_z);
  
  float dt;
  if (!initialized_filter_)
    {
      
      // initialize roll/pitch orientation from acc. vector
  	  //double roll  = atan2(a_y, sqrt(a_x*a_x + a_z*a_z));
  	  //double pitch = atan2(-a_x, sqrt(a_y*a_y + a_z*a_z));
  	  //double yaw = 0.0;

      //tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);

      //X_(1) = init_q.getX();
      //X_(2) = init_q.getY();
      //X_(3) = init_q.getZ();
      //X_(0) = init_q.getW();
      float q0, q1, q2, q3;         
      getMeasurement(a_x, a_y, a_z, mag_fld.x, mag_fld.y, mag_fld.z,
                     q0, q1, q2, q3);
      X_(0) = q0;
      X_(1) = q1;
      X_(2) = q2;
      X_(3) = q3;
      P_ = Eigen::MatrixXf::Zero(4,4);
      
      last_time_filter_ = time;
      initialized_filter_ = true;
    }
    else
     {
      // determine dt: either constant, or from IMU timestamp

      if (constant_dt_ > 0.0)
    	  dt = constant_dt_;
      else 
      	dt = (time - last_time_filter_).toSec();
     
      last_time_filter_ = time;
      
      
      ros::Time t_in, t_out;        
      t_in = ros::Time::now();
      getPrediction(ang_vel.x, ang_vel.y, ang_vel.z, dt);

      if(std::isnan(mag_fld.x) || std::isnan(mag_fld.y) || std::isnan(mag_fld.z))
        update(lin_acc.x, lin_acc.y, lin_acc.z, dt);
      else
        updateWithMag(lin_acc.x, lin_acc.y, lin_acc.z, mag_fld.x, 
                                     mag_fld.y, mag_fld.z, dt);
      t_out = ros::Time::now(); 
      float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
      printf("%.6f\n", dt_tot);

      std_msgs::Float32 acc_mag_msg;
      //acc_mag_msg.data = a_magnitude;
      //acc_mag_publisher_.publish(acc_mag_msg);
      publishFilteredMsg(imu_msg_raw);
      //publishTransform(imu_msg_raw);
   
     }
}
//only for initialization
void EKF::getMeasurement(
    float ax, float ay, float az, 
    float mx, float my, float mz,  
    float& q0_meas, float& q1_meas, float& q2_meas, float& q3_meas)
{
  // q_acc is the quaternion obtained from the acceleration vector representing 
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
  double q0_acc, q1_acc, q2_acc, q3_acc;
    
  // Normalize acceleration vector.
  double a_norm = sqrt(ax*ax + ay*ay + az*az);
    ax /= a_norm;
    ay /= a_norm;
    az /= a_norm;

  if (az == -1)
  {
    q0_acc =  0;
    q1_acc = -1;
    q2_acc =  0;
  }  
  else 
  {
    q0_acc =  sqrt((az + 1) * 0.5);	
    q1_acc = -ay/(2.0 * q0_acc);
    q2_acc =  ax/(2.0 * q0_acc);

  }
  
  
  // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
  // frame by the inverse of q_acc.
  // l = R(q_acc)^-1 m
  double lx = (q0_acc*q0_acc + q1_acc*q1_acc - q2_acc*q2_acc)*mx + 
      2.0 * (q1_acc*q2_acc)*my - 2.0 * (q0_acc*q2_acc)*mz;
  double ly = 2.0 * (q1_acc*q2_acc)*mx + (q0_acc*q0_acc - q1_acc*q1_acc + 
      q2_acc*q2_acc)*my + 2.0 * (q0_acc*q1_acc)*mz;
  
  // q_mag is the quaternion that rotates the Global frame (North West Up) into
  // the intermediary frame. q1_mag and q2_mag are defined as 0.
	double gamma = lx*lx + ly*ly;	
	double beta = sqrt(gamma + lx*sqrt(gamma));
  double q0_mag = beta / (sqrt(2.0 * gamma));  
  double q3_mag = ly / (sqrt(2.0) * beta); 
    
  // The quaternion multiplication between q_acc and q_mag represents the 
  // quaternion, orientation of the Global frame wrt the local frame.  
  // q = q_acc times q_mag 
  
  q0_meas = q0_acc*q0_mag;     
  q1_meas = q1_acc*q0_mag + q2_acc*q3_mag;
  q2_meas = q2_acc*q0_mag - q1_acc*q3_mag;
  q3_meas = q0_acc*q3_mag;
}

void EKF::getPrediction(float wx, float wy, float wz, float dt)
{  
   Eigen::Matrix<float, 4, 4> F;  // State-transition matrix   
   Eigen::Matrix<float, 4, 3> Xi;           
   Eigen::Matrix<float, 4, 4> Q;  // process noise covariance
   Eigen::Matrix3f Sigma_g, I3;
   I3 = Eigen::Matrix3f::Identity(); 
  
   float q0, q1, q2, q3, norm;
   q0 = X_(0);
   q1 = X_(1);
   q2 = X_(2);
   q3 = X_(3);
  
   // compute "a priori" state estimate 
   X_(0) += 0.5 * dt*( wx*q1 + wy*q2 + wz*q3);   
   X_(1) += 0.5 * dt*(-wx*q0 - wy*q3 + wz*q2);
   X_(2) += 0.5 * dt*( wx*q3 - wy*q0 - wz*q1);
   X_(3) += 0.5 * dt*(-wx*q2 + wy*q1 - wz*q0);

      
   //Linear state transition matrix   

   F <<            1,   0.5f *wx*dt,   0.5f *wy*dt,    0.5f *wz*dt,
        -0.5f *wx*dt,             1,   0.5f *wz*dt,   -0.5f *wy*dt,   
        -0.5f *wy*dt,  -0.5f *wz*dt,            1,     0.5f *wx*dt, 
        -0.5f *wz*dt,   0.5f *wy*dt,  -0.5f *wx*dt,              1;
      
   Sigma_g = sigma_g_ * I3; 
   
   Xi <<  X_(1),  X_(2),  X_(3),
         -X_(0), -X_(3),  X_(2),
          X_(3), -X_(0), -X_(1),
         -X_(2),  X_(1), -X_(0);

   
   float dt2 = dt*dt*0.25;
   Q.noalias() = dt2 * (Xi *Sigma_g * Xi.transpose());
   P_ = F*(P_*F.transpose()) + Q;
       
}

void EKF::RotFromQuat(Eigen::Matrix<float, 3, 3>& R)
{
   float q0, q1, q2, q3;
   q0 = X_(0);
   q1 = X_(1);
   q2 = X_(2);
   q3 = X_(3);
   R << (q0*q0 + q1*q1 - q2*q2 - q3*q3),   2*(q1*q2 - q0*q3),         2*(q1*q3 + q0*q2),
         2*(q1*q2 + q0*q3),                q0*q0-q1*q1+q2*q2-q3*q3,   2*(q2*q3 - q0*q1),
         2*(q1*q3 - q0*q2),                2*(q2*q3 + q0*q1),         q0*q0-q1*q1-q2*q2+q3*q3; 
}

void EKF::update(float ax, float ay, float az, float dt)
{
    Eigen::Matrix<float, 3, 4> H;  // Measurement model Jacobian
    Eigen::Matrix<float, 4, 3> K;  // Kalman gain
    Eigen::Matrix<float, 4, 4> I4;  // 4x4 Identity matrix
    Eigen::Matrix3f I3, Sigma_a, R;
    Eigen::Vector3f Z_est;
    I3 = Eigen::Matrix3f::Identity();
    I4 = Eigen::MatrixXf::Identity(4,4);

    Sigma_a = sigma_a_ * I3;
    RotFromQuat(R);
    Z_est = (R * gravity_vec_);
      
    //ROS_WARN("NO_MAG");
    //ROS_INFO("acc vector: %f,%f,%f", ax, ay, az);  
    Eigen::Vector3f Z_meas(ax, ay, az);
    
    // find the matrix H by Jacobian.
    float q0, q1, q2, q3;
    q0 = X_(0);
    q1 = X_(1);
    q2 = X_(2);
    q3 = X_(3);
    H << 9.81*2*q2,  9.81*2*q3,  9.81*2*q0, -9.81*2*q1,
        -9.81*2*q1, -9.81*2*q0,  9.81*2*q3,  9.81*2*q2,
         9.81*2*q0, -9.81*2*q1, -9.81*2*q2,  9.81*2*q3; 
           
    //Compute the Kalman gain K
    
    K = Eigen::MatrixXf::Zero(4,3);
    Eigen::Matrix<float, 3, 3> S;
 
    S=H*(P_*H.transpose()) + Sigma_a;
    K = P_ * H.transpose() * S.inverse();
    
    Eigen::Vector3f Error_vec;
    Error_vec = Z_meas - Z_est;
    //ROS_INFO("error1: %f,%f,%f", Error_vec(0),Error_vec(1),Error_vec(2));
    
        
    // Update State and Covariance  
    X_.noalias() += K * Error_vec;;
    P_ = (I4 - K*H)*P_;
  
    double norm = sqrt(X_(0)*X_(0) + X_(1)*X_(1) + X_(2)*X_(2) + X_(3)*X_(3));
    X_ /= norm;

}

void EKF::updateWithMag(float ax, float ay, float az, 
                        float mx, float my, float mz, float dt)
{
    Eigen::Matrix<float, 6, 4> H;  // Measurement model Jacobian
    Eigen::Matrix<float, 4, 6> K;  // Kalman gain
    Eigen::Matrix<float, 4, 4> I4;  // 4x4 Identity matrix
    Eigen::Matrix<float, 6, 6> Q;  // process noise covariance    
    Eigen::Matrix<float, 6, 1> Z_est, Z_meas, Error_vec;;  // process noise covariance      
    Eigen::Matrix3f I3, R;
    //Eigen::Vector3f Z_est;
    I3 = Eigen::Matrix3f::Identity();
    I4 = Eigen::MatrixXf::Identity(4,4);

    Q = Eigen::MatrixXf::Zero(6,6);
    Q.block<3,3>(0,0)= sigma_a_ * I3; 
    Q.block<3,3>(3,3)= sigma_m_ * I3;
    
    //ROS_WARN("MAG"); 

    RotFromQuat(R);
    Z_est.block<3,1>(0,0) = (R * gravity_vec_);
    Z_est.block<3,1>(3,0) = (R * h_ref_vec_);
    //Z_est << ax, ay, az, mx, my, mz;
    //ROS_INFO("acc vector: %f,%f,%f", ax, ay, az);  
    Z_meas << ax, ay, az, mx, my, mz;
    
    // find the matrix H by Jacobian.
    float q0, q1, q2, q3;
    q0 = X_(0);
    q1 = X_(1);
    q2 = X_(2);
    q3 = X_(3);
    H << 9.81*2*q2,  9.81*2*q3,  9.81*2*q0, -9.81*2*q1,
        -9.81*2*q1, -9.81*2*q0,  9.81*2*q3,  9.81*2*q2,
         9.81*2*q0, -9.81*2*q1, -9.81*2*q2,  9.81*2*q3, 
         2*(mx*q0 + mz*q2 - my*q3),  2*(mx*q1 + my*q2 + mz*q3),  2*(mz*q0 + my*q1 - mx*q2), -2*(my*q0 - mz*q1 + mx*q3),
         2*(my*q0 - mz*q1 + mx*q3), -2*(mz*q0 + my*q1 - mx*q2),  2*(mx*q1 + my*q2 + mz*q3),  2*(mx*q0 + mz*q2 - my*q3),
         2*(mz*q0 + my*q1 - mx*q2),  2*(my*q0 - mz*q1 + mx*q3), -2*(mx*q0 + mz*q2 - my*q3),  2*(mx*q1 + my*q2 + mz*q3);
        
    //Compute the Kalman gain K
    
    K = Eigen::MatrixXf::Zero(4,6);
    Eigen::Matrix<float, 6, 6> S;
 
    S = H*(P_*H.transpose()) + Q;
    K = P_ * H.transpose() * S.inverse();
    
    
    Error_vec = Z_meas - Z_est;
    //ROS_INFO("error1: %f,%f,%f", Error_vec(0),Error_vec(1),Error_vec(2));
    
        
    // Update State and Covariance  
    X_.noalias() += K * Error_vec;;
    
    
    P_ = (I4 - K*H)*P_;

    //q0 = X_(0); q1 = X_(1); q2 = X_(2); q3 = X_(3);
    
    double norm = sqrt(X_(0)*X_(0) + X_(1)*X_(1) + X_(2)*X_(2) + X_(3)*X_(3));
    X_ /= norm;

}


void EKF::publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  double r_raw ,p_raw ,y_raw;
  tf::Quaternion q_raw;
  tf::quaternionMsgToTF(imu_msg_raw->orientation, q_raw);
  tf::Matrix3x3 M;
  M.setRotation(q_raw);
  M.getRPY(r_raw, p_raw, y_raw);

  double r, p, y;
  tf::Quaternion q(X_(1), X_(2), X_(3), X_(0));
  M.setRotation(q);
  M.getRPY(r, p, y);

  tf::Quaternion q_final = tf::createQuaternionFromRPY(r, p, y_raw);
  //tf::Quaternion q(q1, q2, q3, q0);
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
  transform.setRotation( q_final );
  tf_broadcaster_.sendTransform( tf::StampedTransform( transform,
                   imu_msg_raw->header.stamp,
                   fixed_frame_,
                   imu_frame_ ) );

}

void EKF::publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  // create orientation quaternion
  // q0 is the angle, q1, q2, q3 are the axes
  tf::Quaternion q(-X_(1), -X_(2), -X_(3), X_(0));

  // create and publish fitlered IMU message
  boost::shared_ptr<sensor_msgs::Imu> imu_msg = boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);

  imu_msg->header.frame_id = fixed_frame_;
  tf::quaternionTFToMsg(q, imu_msg->orientation);
  imu_EKF_publisher_.publish(imu_msg);

  double roll, pitch, yaw;
  tf::Matrix3x3 M;
  M.setRotation(q);
  M.getRPY(roll, pitch, yaw);
  std_msgs::Float32 roll_msg;
  std_msgs::Float32 pitch_msg;
  std_msgs::Float32 yaw_msg;
  roll_msg.data = roll;
  pitch_msg.data = pitch;
  yaw_msg.data = yaw;
  roll_ekf_publisher_.publish(roll_msg);
  pitch_ekf_publisher_.publish(pitch_msg);
  yaw_ekf_publisher_.publish(yaw_msg);
  //std_msgs::Float32 bias_ax_msg;
  //std_msgs::Float32 bias_ay_msg;
  //std_msgs::Float32 bias_az_msg;
  //bias_ax_msg.data = b_ax;
  //bias_ay_msg.data = b_ay;
  //bias_az_msg.data = b_az;
  

   
  //bias_ax_publisher_.publish(bias_ax_msg);
  //bias_ay_publisher_.publish(bias_ay_msg);  
  //bias_az_publisher_.publish(bias_az_msg);
  
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 

  float a_x = lin_acc.x;
  float a_y = lin_acc.y;
  float a_z = lin_acc.z;
  double roll_acc, pitch_acc;
  roll_acc  = atan2(a_y, sqrt(a_x*a_x + a_z*a_z));
  pitch_acc = atan2(-a_x, sqrt(a_y*a_y + a_z*a_z));
  std_msgs::Float32 roll_acc_msg;
  std_msgs::Float32 pitch_acc_msg;
  roll_acc_msg.data = roll_acc;
  pitch_acc_msg.data = pitch_acc;
  roll_acc_publisher_.publish(roll_acc_msg);
  pitch_acc_publisher_.publish(pitch_acc_msg);
/*
   // XSens roll and pitch publishing 
   tf::Quaternion q_kf; 
   tf::quaternionMsgToTF(imu_msg_raw->orientation, q_kf);
   M.setRotation(q_kf);
   M.getRPY(roll, pitch, y);
   roll_msg.data = roll;
   pitch_msg.data = pitch;
   roll_kf_publisher_.publish(roll_msg);
   pitch_kf_publisher_.publish(pitch_msg);*/
}









  

