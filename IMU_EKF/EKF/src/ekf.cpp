
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
  
  int queue = 10;

  imu_EKF_publisher_ = nh_.advertise<sensor_msgs::Imu>(
	    "imu_EKF/data", queue);

  roll_ekf_publisher_    = nh.advertise<std_msgs::Float32>("roll_ekf", queue);
  pitch_ekf_publisher_   = nh.advertise<std_msgs::Float32>("pitch_ekf", queue);
  yaw_ekf_publisher_     = nh.advertise<std_msgs::Float32>("yaw_ekf", queue);
  bias_ax_publisher_     = nh.advertise<std_msgs::Float32>("bias_ax", queue);
  bias_ay_publisher_     = nh.advertise<std_msgs::Float32>("bias_ay", queue);
  bias_az_publisher_     = nh.advertise<std_msgs::Float32>("bias_az", queue);

  bias_gx_publisher_     = nh.advertise<std_msgs::Float32>("bias_gx", queue);
  bias_gy_publisher_     = nh.advertise<std_msgs::Float32>("bias_gy", queue);
  bias_gz_publisher_     = nh.advertise<std_msgs::Float32>("bias_gz", queue);

  roll_acc_publisher_    = nh.advertise<std_msgs::Float32>("roll_acc", queue);
  pitch_acc_publisher_   = nh.advertise<std_msgs::Float32>("pitch_acc", queue);
  yaw_g_publisher_   = nh.advertise<std_msgs::Float32>("yaw_g", queue);
  yaw_m_publisher_   = nh.advertise<std_msgs::Float32>("yaw_m", queue);
  //acc_mag_publisher_ = nh.advertise<std_msgs::Float32>("acc_mag", queue);
  //g_mag_publisher_   = nh.advertise<std_msgs::Float32>("g_mag", queue);

  // **** register subscribers
  int queue_size = 5;

//  imu_subscriber_ = nh_.subscribe(
//    "imu/data_raw", queue_size, &EKF::imuCallback, this);
 

imu_subscriber_.reset(new ImuSubscriber(
    nh_, "imu/data_raw", queue_size));

mag_subscriber_.reset(new MagSubscriber(
      nh_, "imu/mag", queue_size));

sync_.reset(new Synchronizer(
      SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
    sync_->registerCallback(boost::bind(&EKF::imuMagCallback, this, _1, _2));
}


EKF::~EKF()
{
  ROS_INFO ("Destroying EKF");
}

void EKF::initializeParams()
{
   //last_time_ = 0.0;
 // initialized_ = false;
  if (!nh_private_.getParam ("sigma_gx", sigma_g_))
    sigma_g_ = 0.01;
  
  if (!nh_private_.getParam ("sigma_a", sigma_a_))
    sigma_a_ = 0.01;
  
  if (!nh_private_.getParam ("constant_dt", constant_dt_))
      constant_dt_ = 0.0;
  if (!nh_private_.getParam ("gain", gain_))
     gain_ = 0.1;
  if (!nh_private_.getParam ("sigma_bax", sigma_bax_))
    sigma_bax_ = 0.01;
  if (!nh_private_.getParam ("sigma_bay", sigma_bay_))
    sigma_bay_ = 0.01;
  if (!nh_private_.getParam ("sigma_baz", sigma_baz_))
    sigma_baz_ = 0.01;
  if (!nh_private_.getParam ("sigma_bgx", sigma_bgx_))
    sigma_bgx_ = 0.01;
  if (!nh_private_.getParam ("sigma_bgy", sigma_bgy_))
    sigma_bgy_ = 0.01;
  if (!nh_private_.getParam ("sigma_bgz", sigma_bgz_))
    sigma_bgz_ = 0.01;
  if (!nh_private_.getParam ("hx", hx_))
    hx_ = 0.2;
  if (!nh_private_.getParam ("hy", hy_))
    hy_ = 0.046;
  if (!nh_private_.getParam ("hz", hz_))
    hz_ = -0.48;
  if (!nh_private_.getParam ("sigma_h", sigma_h_))
    sigma_h_ = 0.01;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
     fixed_frame_ = "odom";
  if (!nh_private_.getParam ("imu_frame", imu_frame_))
       imu_frame_ = "imu_niac";
  if (!nh_private_.getParam ("threshold_gyro_step", threshold_g_))
       threshold_g_ = 0.001;
  if (!nh_private_.getParam ("threshold_acc_step", threshold_a_))
       threshold_a_ = 0.001;
  if (!nh_private_.getParam ("alpha", alpha_))
       alpha_ = 1.0;
  gravity_vec_ = Eigen::Vector3f(0, 0, 9.81);
}

void EKF::imuMagCallback(
  const ImuMsg::ConstPtr& imu_msg_raw,
  const MagMsg::ConstPtr& mag_msg)
{
  boost::mutex::scoped_lock(mutex_);
  
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& mag_fld = mag_msg->vector;
  
  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

//void EKF::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
//{

//  boost::mutex::scoped_lock(mutex_);

    
//  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
//  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 

//  ros::Time time = imu_msg_raw->header.stamp;
  

  float a_x = lin_acc.x;
  float a_y = lin_acc.y;
  float a_z = lin_acc.z;
  float mx = mag_fld.x;
  float my = mag_fld.y;
  float mz = mag_fld.z;

  float dt;
  if (!initialized_filter_)
    {
      
      // initialize roll/pitch orientation from acc. vector
  	  double roll  = atan2(a_y, sqrt(a_x*a_x + a_z*a_z));
  	  double pitch = atan2(-a_x, sqrt(a_y*a_y + a_z*a_z));
  	  double yaw = atan2( (-my*cos(roll) + mz*sin(roll) ) , (mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll)) );
      tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);
      
      //INITIALIZE STATE

      q1 = init_q.getX();
      q2 = init_q.getY();
      q3 = init_q.getZ();
      q0 = init_q.getW();
      b_ax = 0.0;
      b_ay = 0.0;
      b_az = 0.0;

      b_gx_ = 0.0;
      b_gy_ = 0.0;
      b_gz_ = 0.0; 
      p_prev_ = 0; 
      q_prev_ = 0;
      r_prev_ = 0;
      ax_prev_ = 0; 
      ay_prev_ = 0;
      az_prev_ = 0;
      //INITIALIZE COVARIANCE
      yaw_g = 0.0;
      //P = Eigen::MatrixXf::Identity(7, 7);
      P = Eigen::MatrixXf::Zero(7, 7);

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
    

    filter(ang_vel.x, ang_vel.y, ang_vel.z,
    lin_acc.x, lin_acc.y, lin_acc.z,
    mag_fld.x, mag_fld.y, mag_fld.z,
    dt);
    
    std_msgs::Float32 acc_mag_msg;
    //acc_mag_msg.data = a_magnitude;
    //acc_mag_publisher_.publish(acc_mag_msg);
    publishFilteredMsg(imu_msg_raw);
    //publishTransform(imu_msg_raw);
   }
}

void EKF::filter(float p, float q, float r, 
  float ax, float ay, float az,
  float mx, float my, float mz, 
  float dt)
{

  /************************** PREDICTION *****************************/

  
  Eigen::Matrix<float, 7, 7> F;  // State-transition matrix             
  Eigen::Matrix<float, 7, 7> Q;  // process noise covariance
  Eigen::Matrix<float, 4, 3> Xi; // quaternion covariance 
  Eigen::Matrix<float, 4, 4> W;
  Eigen::Matrix<float, 6, 7> H;  // Measurement model Jacobian
  Eigen::Matrix<float, 7, 6> K;  // Kalman gain
  Eigen::Matrix<float, 6, 6> R;
  Eigen::Matrix3f Sigma_g, Sigma_a, Sigma_h, Sigma_ba, Sigma_bg, C_bn, I3;
  I3 = Eigen::MatrixXf::Identity(3,3);

  //Eigen::Vector3f Z_est;
  Eigen::Matrix<float, 6, 1> Z_est;  
  Eigen::Vector3f bias_vec;
  float wx_hut, wy_hut, wz_hut;
  double c,s, j;
  c=1; s=1;
  //float pdt, qdt, rdt;
  float norm;
  float w_norm, m_norm;
  yaw_g = yaw_g + r*dt; 
   // compute "a priori" state estimate 
  wx_hut = p-b_gx_; 
  wy_hut = q-b_gy_; 
  wz_hut = r-b_gz_;

  //q1 += 0.5 * dt * ( wx_hut*q0 - wy_hut*q3 + wz_hut*q2);
  //q2 += 0.5 * dt * ( wx_hut*q3 + wy_hut*q0 - wz_hut*q1);
  //q3 += 0.5 * dt * (-wx_hut*q2 + wy_hut*q1 + wz_hut*q0);
  //q0 += 0.5 * dt * (-wx_hut*q1 - wy_hut*q2 - wz_hut*q3);  

   
  m_norm = sqrt(mx*mx + my*my+mz*mz);
   w_norm = sqrt(wx_hut*wx_hut+wy_hut*wy_hut+wz_hut*wz_hut);
   //j = 1-sqrt(q1*q1 + q2*q2 + q3*q3 + q0*q0);

   if (w_norm >0)
   {
    s = (sin(w_norm*dt*0.5))/(w_norm*dt*0.5);
    c = cos(w_norm*dt*0.5);
    }
  ROS_INFO("c, s: %f,%f", c, s);
    
ROS_INFO("norm, dt, c, s: %f,%f,%f,%f", w_norm, dt,c,s);
//c + lambda_*j*dt
  q1 = q1*c + 0.5*dt*s*( wx_hut*q0 - wy_hut*q3 + wz_hut*q2);
  q2 = q2*c + 0.5*dt*s*( wx_hut*q3 + wy_hut*q0 - wz_hut*q1);
  q3 = q3*c + 0.5*dt*s*(-wx_hut*q2 + wy_hut*q1 + wz_hut*q0);
  q0 = q0*c + 0.5*dt*s*(-wx_hut*q1 - wy_hut*q2 - wz_hut*q3);


 
 // ROS_INFO("q: %f,%f,%f,%f", q1, q2, q3, q0);
  ROS_INFO("MAG_NORM: %f", m_norm);
  bias_vec << b_ax, b_ay, b_az;

  X << q1,
       q2,
       q3,
       q0,
       b_ax,
       b_ay,
       b_az;

  // K = Eigen::MatrixXf::Zero(10,3);
 
  F <<                  1,   0.5f *(r-b_gz_)*dt,   -0.5f *(q-b_gy_)*dt,  0.5f *(p-b_gx_)*dt,     0, 0, 0, 
       -0.5f *(r-b_gz_)*dt,                   1,    0.5f *(p-b_gx_)*dt,  0.5f *(q-b_gy_)*dt,     0, 0, 0,  
        0.5f *(q-b_gy_)*dt,  -0.5f *(p-b_gx_)*dt,                    1,  0.5f *(r-b_gz_)*dt,     0, 0, 0,
       -0.5f *(p-b_gx_)*dt,  -0.5f *(q-b_gy_)*dt,   -0.5f *(r-b_gz_)*dt,                  1,     0, 0, 0,
                        0,                   0,                    0,                     0,     1, 0, 0,
                        0,                   0,                    0,                     0,     0, 1, 0,
                        0,                   0,                    0,                     0,     0, 0, 1;



  Sigma_a = sigma_a_ * I3;
  Sigma_g = sigma_g_ * I3;
  Sigma_h = sigma_h_ * I3;
  
  Sigma_ba <<   sigma_bax_, 0, 0,            
                0, sigma_bay_, 0,
                0,        0, sigma_baz_;
    
  Xi << q0, -q3,  q2,
        q3,  q0, -q1,
       -q2,  q1,  q0,
       -q1, -q2, -q3;

  float dt2 = dt*dt*0.25;
  
  W.noalias() = dt2 * (Xi *Sigma_g * Xi.transpose());
   
  Q = Eigen::MatrixXf::Zero(7,7);
  Q.block<4,4>(0,0)= W; Q.block<3,3>(4,4)=(dt * Sigma_ba ); 
 
//std::cout << "P" << P << std::endl;
//std::cout << "Q" << Q << std::endl;
   
// compute "a priori" covariance matrix (P)
    
  P = (F*(P*F.transpose())) + Q;
 
    // CORRECTION /

    
   C_bn <<  (q1*q1-q2*q2-q3*q3+q0*q0),   2*(q1*q2+q3*q0),                    2*(q1*q3-q2*q0),
            2*(q1*q2-q3*q0),            -q1*q1+q2*q2-q3*q3+q0*q0,            2*(q2*q3+q1*q0),
            2*(q1*q3+q2*q0),             2*(q2*q3-q1*q0),           -q1*q1-q2*q2+q3*q3+q0*q0; 

    //Z_est = (C_bn*gravity_vec_) + bias_vec;


  Z_est <<                                                     2*(q1*q3-q2*q0)*9.81,// + b_ax,
                                                               2*(q2*q3+q1*q0)*9.81,// + b_ay,
                                                    (-q1*q1-q2*q2+q3*q3+q0*q0)*9.81,//+ b_az,
                0.13,//(q1*q1-q2*q2-q3*q3+q0*q0)*hx_ + 2*(q1*q2+q3*q0)*hy_ + 2*(q1*q3-q2*q0)*hz_,
                0.2,//2*(q1*q2-q3*q0)*hx_ + (-q1*q1+q2*q2-q3*q3+q0*q0)*hy_ + 2*(q2*q3+q1*q0)*hz_,
                0.1;//2*(q1*q3+q2*q0)*hx_ + 2*(q2*q3-q1*q0)*hy_ + (-q1*q1-q2*q2+q3*q3+q0*q0)*hz_;


   ROS_INFO("ROTATED H: %f,%f,%f", Z_est(0), Z_est(1), Z_est(2));
 
     Eigen::Matrix<float, 6, 1>   Z_meas, fields;
     Z_meas << ax, ay, az, 0.13,0.2,0.1;//mx, my, mz;

    
    std::cout << "Z_est" << Z_est << std::endl;
    //  Jacobian matrix (H) of Z_est.

    H <<                                                                       9.81*2*q3, -9.81*2*q0, 9.81*2*q1, -9.81*2*q2, 0, 0, 0,
                                                                               9.81*2*q0,  9.81*2*q3, 9.81*2*q2,  9.81*2*q1, 0, 0, 0,
                                                                              -9.81*2*q1, -9.81*2*q2, 9.81*2*q3,  9.81*2*q0, 0, 0, 0,
        2*q1*hx_+2*q2*hy_+2*q3*hz_, -2*q2*hx_+2*q1*hy_-2*q0*hz_,  -2*q3*hx_+2*q0*hy_+2*q1*hz_,  2*q0*hx_+2*q3*hy_-2*q2*hz_,  0, 0, 0,
        2*q2*hx_-2*q1*hy_+2*q0*hz_,  2*q1*hx_+2*q2*hy_+2*q3*hz_, -2*q0*hx_- 2*q3*hy_+2*q2*hz_, -2*q3*hx_+2*q0*hy_+2*q1*hz_,  0, 0, 0,  
        2*q3*hx_-2*q0*hy_-2*q1*hz_,  2*q0*hx_+2*q3*hy_-2*q0*hz_,   2*q1*hx_+2*q2*hy_+2*q3*hz_,  2*q2*hx_-2*q1*hy_+2*q0*hz_,  0, 0, 0;
  


    R = Eigen::MatrixXf::Zero(6,6);
    R.block<3,3>(0,0) = Sigma_a; R.block<3,3>(3,3) = Sigma_h; 
    //R = Sigma_a;
    //std::cout << "R" << R << std::endl;
    //Compute the Kalman gain K
    
    Eigen::Matrix<float, 6, 6> S;
    
    S = H*P*H.transpose() + R;
    K = P*H.transpose()*S.inverse();
    //std::cout << "K" << K << std::endl;
   
    //Eigen::Vector3f Error_vec;
    Eigen::Matrix<float, 6, 1> Error_vec;

  if (isnan(mx))
  {
    Error_vec << ax - Z_est(0),
                 ay - Z_est(1),
                 az - Z_est(2),
                            0,  
                            0,
                            0;  
  }
  else
    Error_vec = Z_meas - Z_est;
    //ROS_INFO("error1: %f,%f,%f,%f,%f,%f", Error_vec(0),Error_vec(1),Error_vec(2), Error_vec(3), Error_vec(4), Error_vec(5));
   
    // Update State and Covariance  
    X.noalias() += K * Error_vec;
    
  /*
    if ( fabs(q - q_prev_)> threshold_a_ || fabs(q-b_gy_)>0.1 || fabs(ay - ay_prev_) > threshold_a_ || fabs(ax) > 1)
      X(4) = b_ax;
    if (fabs(ay - ay_prev_)/dt > threshold_a_ || fabs(p-b_gx_)>0.4)
      X(5) = b_ay;
    if (fabs(az - az_prev_)/dt > threshold_a_)
      X(6) = b_az;
    ROS_INFO("step, th: %f,%f", fabs(ax - ax_prev_),threshold_a_);
   ax_prev_ = ax; 
   ay_prev_ = ay;
   az_prev_ = az;  
 
    P.noalias() -= K*(H*P);
  */  
   //}   
  
    //std::cout << "P" << P << std::endl;

 
    q1 = X(0); q2 = X(1); q3 = X(2); q0 = X(3);
    b_ax = X(4); b_ay = X(5); b_az = X(6);
    
  
    norm = invSqrt(q1*q1+q2*q2+q3*q3+q0*q0);
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
    q0 *= norm;

    if (fabs(p - p_prev_)< threshold_g_ && fabs(p-b_gx_)<0.5)
      b_gx_ = b_gx_ + alpha_ * (p - b_gx_);
    if (fabs(q - q_prev_)< threshold_g_ && fabs(q-b_gy_)<0.5)
      b_gy_ = b_gy_ + alpha_ * (q - b_gy_);

    if (fabs(r - r_prev_)< threshold_g_ && fabs(r-b_gz_)<0.5)
      b_gz_ = b_gz_ + alpha_ * (r - b_gz_);
    
    p_prev_ = p; q_prev_ = q; r_prev_ = r;
 
    ax_ = ax; ay_ = ay; az_ = az;
    mx_ = mx; my_ = my; mz_ = mz;
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
  tf::Quaternion q(q1, q2, q3, q0);
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
  tf::Quaternion q(q1, q2, q3, q0);

  // create and publish fitlered IMU message
  boost::shared_ptr<sensor_msgs::Imu> imu_msg = boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);

  imu_msg->header.frame_id = fixed_frame_;
  tf::quaternionTFToMsg(q, imu_msg->orientation);
  imu_EKF_publisher_.publish(imu_msg);

  double roll, pitch, yaw, yaw_m;
  tf::Matrix3x3 M;
  M.setRotation(q);
  M.getRPY(roll, pitch, yaw);
  std_msgs::Float32 roll_msg;
  std_msgs::Float32 pitch_msg;
  std_msgs::Float32 yaw_msg;
  std_msgs::Float32 yaw_g_msg;
  std_msgs::Float32 yaw_m_msg;

  roll_msg.data = roll;
  pitch_msg.data = pitch;
  yaw_msg.data = yaw;
  yaw_g_msg.data = yaw_g;

  double rolla  = atan2(ay_, sqrt(ax_*ax_ + az_*az_));
  double pitcha = atan2(-ax_, sqrt(ay_*ay_ + az_*az_));
  yaw_m =atan2( (-my_*cos(rolla) + mz_*sin(rolla) ) , (mx_*cos(pitcha) + my_*sin(pitcha)*sin(rolla) + mz_*sin(pitcha)*cos(rolla)) );
  //yaw_m =atan2( (mx_*sin(roll)*sin(pitch) + my_*cos(roll) - mz_*sin(roll)*sin(pitch)),(mx_*cos(pitch)+ mz_*sin(pitch)) );
  yaw_m_msg.data = yaw_m;

  roll_ekf_publisher_.publish(roll_msg);
  std_msgs::Float32 bias_ax_msg;
  std_msgs::Float32 bias_ay_msg;
  std_msgs::Float32 bias_az_msg;

  std_msgs::Float32 bias_gx_msg;
  std_msgs::Float32 bias_gy_msg;
  std_msgs::Float32 bias_gz_msg;

  bias_ax_msg.data = b_ax;
  bias_ay_msg.data = b_ay;
  bias_az_msg.data = b_az;
  
  bias_gx_msg.data = b_gx_;
  bias_gy_msg.data = b_gy_;
  bias_gz_msg.data = b_gz_;
  
  pitch_ekf_publisher_.publish(pitch_msg);
  yaw_ekf_publisher_.publish(yaw_msg);
  yaw_g_publisher_.publish(yaw_g_msg);
  yaw_m_publisher_.publish(yaw_m_msg);  
  
  bias_ax_publisher_.publish(bias_ax_msg);
  bias_ay_publisher_.publish(bias_ay_msg);  
  bias_az_publisher_.publish(bias_az_msg);
  
  bias_gx_publisher_.publish(bias_gx_msg);
  bias_gy_publisher_.publish(bias_gy_msg);  
  bias_gz_publisher_.publish(bias_gz_msg);

  
    
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









  

