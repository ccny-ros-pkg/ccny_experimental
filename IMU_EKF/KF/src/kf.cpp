
#include "KF/kf.h"
#include <iostream>

KF::KF(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  initialized_filter_(false),
  initialized_LP_filter_(false),
  q1_(0.0), q2_(0.0), q3_(0.0),  q4_(1.0)
{
  ROS_INFO ("Starting KF");
  initializeParams();
  
  int queue_size = 5;

  imu_KF_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu_KF/data", queue_size);

  roll_ekf_publisher_    = nh_.advertise<std_msgs::Float32>("roll_ekf", queue_size);
  pitch_ekf_publisher_   = nh_.advertise<std_msgs::Float32>("pitch_ekf", queue_size);
  yaw_ekf_publisher_     = nh_.advertise<std_msgs::Float32>("yaw_ekf", queue_size);

  roll_xekf_publisher_    = nh_.advertise<std_msgs::Float32>("roll_xekf", queue_size);
  pitch_xekf_publisher_   = nh_.advertise<std_msgs::Float32>("pitch_xekf", queue_size);
  yaw_xekf_publisher_     = nh_.advertise<std_msgs::Float32>("yaw_xekf", queue_size);

  bias_gx_publisher_     = nh_.advertise<std_msgs::Float32>("bias_gx", queue_size);
  bias_gy_publisher_     = nh_.advertise<std_msgs::Float32>("bias_gy", queue_size);
  bias_gz_publisher_     = nh_.advertise<std_msgs::Float32>("bias_gz", queue_size);

  roll_acc_publisher_    = nh_.advertise<std_msgs::Float32>("roll_acc", queue_size);
  pitch_acc_publisher_   = nh_.advertise<std_msgs::Float32>("pitch_acc", queue_size);
  yaw_g_publisher_   = nh_.advertise<std_msgs::Float32>("yaw_g", queue_size);
  yaw_m_publisher_   = nh_.advertise<std_msgs::Float32>("yaw_m", queue_size);
  //acc_mag_publisher_ = nh.advertise<std_msgs::Float32>("acc_mag", queue);
  //g_mag_publisher_   = nh.advertise<std_msgs::Float32>("g_mag", queue);

  // **** register subscribers

//  imu_subscriber_ = nh_.subscribe(
//    "imu/data_raw", queue_size, &EKF::imuCallback, this);

  imu_subscriber_.reset(new ImuSubscriber(
      nh_, "imu/data_raw", queue_size));

  if (use_mag_)
  {
    mag_subscriber_.reset(new MagSubscriber(
        nh_, "imu/mag", queue_size));

    sync_.reset(new Synchronizer(
        SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
    sync_->registerCallback(boost::bind(&KF::imuMagCallback, this, _1, _2));
  }
  else
  {
    imu_subscriber_->registerCallback(&KF::imuCallback, this);
  }
}

KF::~KF()
{
  ROS_INFO ("Destroying KF");
}

void KF::initializeParams()
{
  if (!nh_private_.getParam ("use_mag", use_mag_))
    use_mag_ = false;
  if (!nh_private_.getParam ("do_bias_estimation", do_bias_estimation_))
    do_bias_estimation_ = true;
  
  if (!nh_private_.getParam ("sigma_gx", sigma_gx_))
    sigma_gx_ = 0.01;
  if (!nh_private_.getParam ("sigma_gy", sigma_gy_))
    sigma_gy_ = 0.01;
  if (!nh_private_.getParam ("sigma_gz", sigma_gz_))
    sigma_gz_ = 0.01;
  if (!nh_private_.getParam ("sigma_ax", sigma_ax_))
    sigma_ax_ = 0.01;
  if (!nh_private_.getParam ("sigma_ay", sigma_ay_))
    sigma_ay_ = 0.01;
  if (!nh_private_.getParam ("sigma_ax", sigma_az_))
    sigma_az_ = 0.01;  
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
  if (!nh_private_.getParam ("sigma_hx", sigma_hx_))
    sigma_hx_ = 0.01;
  if (!nh_private_.getParam ("sigma_hx", sigma_hy_))
    sigma_hy_ = 0.01;
  if (!nh_private_.getParam ("sigma_hx", sigma_hz_))
    sigma_hz_ = 0.01;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("imu_frame", imu_frame_))
   imu_frame_ = "imu_niac";
  if (!nh_private_.getParam ("threshold_gyro_step", threshold_g_))
    threshold_g_ = 0.001;
  if (!nh_private_.getParam ("threshold_acc_step", threshold_a_))
    threshold_a_ = 0.001;
  if (!nh_private_.getParam ("alpha", alpha_))
    alpha_ = 0.005;

  gravity_vec_ =  Eigen::Vector3d(0, 0, kGravity);
  magnetic_vec_ = Eigen::Vector3d(hx_, hy_, hz_);
}

void KF::imuCallback(const ImuMsg::ConstPtr& imu_msg_raw)
{
  boost::mutex::scoped_lock(mutex_);

  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
    
  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  double p = ang_vel.x;
  double q = ang_vel.y;
  double r = ang_vel.z;

  double ax = lin_acc.x;
  double ay = lin_acc.y;
  double az = lin_acc.z;

  double dt;

  if (!initialized_filter_)
  {
    // initialize roll/pitch orientation from acc. vector
	  double roll  = atan2(ay, sqrt(ax*ax + az*az));
	  double pitch = atan2(-ax, sqrt(ay*ay + az*az));
	  double yaw = 0.0;
    tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    
    // INITIALIZE STATE
    q1_ = init_q.getX();
    q2_ = init_q.getY();
    q3_ = init_q.getZ();
    q4_ = init_q.getW();
    
    // INITIALIZE VARIABLES
    b_gx_ = 0.0;
    b_gy_ = 0.0;
    b_gz_ = 0.0; 
    p_prev_ = 0; 
    q_prev_ = 0;
    r_prev_ = 0;
    ax_prev_ = 0; 
    ay_prev_ = 0;
    az_prev_ = 0;
    yaw_g_ = 0.0;

    // INITIALIZE COVARIANCES
    Sigma_g_ << sigma_gx_*sigma_gx_, 0, 0,            
                0, sigma_gy_*sigma_gy_, 0,
                0,        0,             0;

    Sigma_a_ << sigma_ax_*sigma_ax_, 0, 0,            
                0, sigma_ay_*sigma_ay_, 0,
                0, 0, sigma_az_*sigma_az_;

    P_ = Eigen::MatrixXd::Zero(4, 4);

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

    yaw_g_ = yaw_g_ + r*dt;

    if (do_bias_estimation_)
      updateBiases(ax, ay, az, p, q, r);
    prediction(p-b_gx_, q-b_gy_, r - b_gz_, dt);
    correctionNoMag(lin_acc.x, lin_acc.y, lin_acc.z);
    
    publishFilteredMsg(imu_msg_raw);
   }
}

void KF::imuMagCallback(
  const ImuMsg::ConstPtr& imu_msg_raw,
  const MagMsg::ConstPtr& mag_msg)
{
  boost::mutex::scoped_lock(mutex_);
  
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& mag_fld = mag_msg->vector;
  
  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;
  
  double p = ang_vel.x;
  double q = ang_vel.y;
  double r = ang_vel.z;
  
  double ax = lin_acc.x;
  double ay = lin_acc.y;
  double az = lin_acc.z;

  double mx = mag_fld.x;
  double my = mag_fld.y;
  double mz = mag_fld.z;

  if (!initialized_filter_)
  {   
    // initialize roll/pitch orientation from acc. vector
	  double roll  = atan2(ay, sqrt(ax*ax + az*az));
	  double pitch = atan2(-ax, sqrt(ay*ay + az*az));
	  double yaw = atan2( (-my*cos(roll) + mz*sin(roll) ) , (mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll)) );
    tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    
    //INITIALIZE STATE
    q1_ = init_q.getX();
    q2_ = init_q.getY();
    q3_ = init_q.getZ();
    q4_ = init_q.getW();
    
    // INITIALIZE VARIABLES
    b_gx_ = 0.0;
    b_gy_ = 0.0;
    b_gz_ = 0.0; 
    p_prev_ = 0; 
    q_prev_ = 0;
    r_prev_ = 0;
    ax_prev_ = 0; 
    ay_prev_ = 0;
    az_prev_ = 0;
    yaw_g_ = yaw;

    h_norm_ = sqrt(hx_*hx_ + hy_*hy_ + hz_*hz_);
    if(1) {
      hx_ /= h_norm_;
      hy_ /= h_norm_;
      hz_ /= h_norm_;
    }

    // INITIALIZE COVARIANCES
    Sigma_g_ << sigma_gx_*sigma_gx_, 0, 0,            
                0, sigma_gy_*sigma_gy_, 0,
                0, 0, sigma_gz_*sigma_gz_;

    Sigma_a_ << sigma_ax_*sigma_ax_, 0, 0,            
                0, sigma_ay_*sigma_ay_, 0,
                0, 0, sigma_az_*sigma_az_;

    Sigma_h_ << sigma_hx_*sigma_hx_, 0, 0,            
                0, sigma_hy_*sigma_hy_, 0,
                0, 0, sigma_hz_*sigma_hz_;
    
    P_ = Eigen::MatrixXd::Zero(4, 4);

    // initialize time
    last_time_filter_ = time;
    initialized_filter_ = true;
  }
  else
  {
    // determine dt: either constant, or from IMU timestamp
    if (isnan(mx) || isnan(my) || isnan(mz))
      return;  
    double dt;
    if (constant_dt_ > 0.0)
  	  dt = constant_dt_;
    else 
    	dt = (time - last_time_filter_).toSec();
    last_time_filter_ = time;
    
    yaw_g_ = yaw_g_ + r*dt;
    
    if (do_bias_estimation_)
      updateBiases(ax, ay, az, p, q, r);

    prediction(p - b_gx_, q - b_gy_, r - b_gz_, dt);
   if (isnan(mx) || isnan(my) || isnan(mz))
    {
      ROS_INFO("Correction (no mag)"); 
      correctionNoMag(ax, ay, az);
    }
    else 
    {
      ROS_INFO("Correction (with mag)");         
      correctionWithMag(ax, ay, az, mx, my, mz);
    }
/*
    if (isnan(mx) || isnan(my) || isnan(mz))
    {
      ROS_INFO("Correction (no mag)"); 
      correctionNoMag(ax, ay, az);
    }
    else 
    {
      double thetaDip_est = getInclination(ax, ay, az, mx, my, mz);
      double thetaDip_true = acos(gravity_vec_.dot(magnetic_vec_)/(h_norm_*kGravity));
      double m_norm = sqrt(mx * mx + my * my + mz * mz); 
      if (fabs(thetaDip_est - thetaDip_true) < Deviation && fabs(m_norm - 0.52) < 1.1)
      {
        ROS_INFO("Correction (with mag)");         
        correctionWithMag(ax, ay, az, mx, my, mz);
      }      
      else
      {
        ROS_INFO("Correction (no mag)");         
        correctionNoMag(ax, ay, az);
      } 
    }*/
    mx_ = mx; my_ = my; mz_ = mz;
    publishFilteredMsg(imu_msg_raw);
    
  }
}

void KF::updateBiases(double ax, double ay, double az, double p, double q, double r) 
{
  float acceleration_magnitude = sqrt(ax*ax + ay*ay + az*az);
  if (fabs(p - p_prev_) < kDeltaAngularVelocityThreshold && 
      fabs(p - b_gx_) < kAngularVelocityThreshold && 
      fabs(acceleration_magnitude - kGravity) < kAccelerationThreshold)
    b_gx_ = b_gx_ + alpha_ * (p - b_gx_);
  if (fabs(q - q_prev_) < kDeltaAngularVelocityThreshold && 
      fabs(q - b_gy_) < kAngularVelocityThreshold && 
      fabs(acceleration_magnitude - kGravity) < kAccelerationThreshold)
    b_gy_ = b_gy_ + alpha_ * (q - b_gy_);
  if (fabs(r - r_prev_) < threshold_g_ && 
      fabs(r - b_gz_) < kAngularVelocityThreshold && 
      fabs(acceleration_magnitude - kGravity) < kAccelerationThreshold)
    b_gz_ = b_gz_ + alpha_ * (r - b_gz_);

  p_prev_ = p; q_prev_ = q; r_prev_ = r;
}

double KF::getInclination(double ax, double ay, double az, double mx, double my, double mz)
{
  Eigen::Vector3d g_est, h_est;
  g_est << 2*(q1_*q3_+q2_*q4_)*ax,
           2*(q2_*q3_-q1_*q4_)*ay,
           (-q1_*q1_ - q2_*q2_ + q3_*q3_ + q4_*q4_)*az,
  
  h_est << (q1_*q1_ - q2_*q2_ - q3_*q3_ + q4_*q4_)*mx + 2*(q1_*q2_ - q3_*q4_)*my + 2*(q1_*q3_ + q2_*q4_)*mz ,
           2*(q1_*q2_ + q3_*q4_)*mx + (-q1_*q1_ + q2_*q2_ - q3_*q3_ + q4_*q4_)*my + 2*(q2_*q3_ - q1_*q4_)*mz,
           2*(q1_*q3_ - q2_*q4_)*mx + 2*(q2_*q3_ + q1_*q4_)*my + (-q1_*q1_ - q2_*q2_ + q3_*q3_ + q4_*q4_)*mz;
  double m_norm = sqrt(mx*mx + my*my + mz*mz);
  double a_norm = sqrt(ax*ax + ay*ay + az*az);
    
  //printf("a_norm, m_norm: %f %f\n", a_norm, m_norm);

  if (a_norm != 0.0  && m_norm != 0.0)
  {
    return acos(g_est.dot(h_est)/(m_norm * a_norm));
  }
  else 
    return Deviation;  
  
}
  
           
void KF::prediction(double p, double q, double r, double dt)
{
  Eigen::Matrix<double, 4, 4> F;  // State-transition matrix             
  Eigen::Matrix<double, 4, 4> Q;  // process noise covariance
  Eigen::Matrix<double, 4, 3> Xi; // quaternion covariance 
  Eigen::Matrix<double, 4, 4> W;
  Eigen::Matrix3d C_bn;
     
  double c = 1; 
  double s = 1;

  double w_norm;
    
  w_norm = sqrt(p * p + q * q + r * r);

  if (w_norm > 0)
  {
    s = (sin(w_norm*dt*0.5))/(w_norm*dt*0.5);
    c = cos(w_norm*dt*0.5);
  }

  q1_ = q1_*c + 0.5*dt*s*( p*q4_ - q*q3_ + r*q2_);
  q2_ = q2_*c + 0.5*dt*s*( p*q3_ + q*q4_ - r*q1_);
  q3_ = q3_*c + 0.5*dt*s*(-p*q2_ + q*q1_ + r*q4_);
  q4_ = q4_*c + 0.5*dt*s*(-p*q1_ - q*q2_ - r*q3_);

  X_ << q1_, q2_, q3_, q4_;
 
  F <<              1,  0.5 * r * dt,  -0.5 * q * dt,  0.5 * p * dt,  
       -0.5 * r * dt,              1,   0.5 * p * dt,  0.5 * q * dt,  
        0.5 * q * dt,  -0.5 * p * dt,              1,  0.5 * r * dt,
       -0.5 * p * dt,  -0.5 * q * dt,  -0.5 * r * dt,             1;
                       
  Xi << q4_, -q3_,  q2_,
        q3_,  q4_, -q1_,
       -q2_,  q1_,  q4_,
       -q1_, -q2_, -q3_;
  
  double dt2 = dt * dt * 0.25;
  W = dt2 * (Xi *Sigma_g_ * Xi.transpose());
  Q = W;

  // compute "a priori" covariance matrix (P)   
  P_ = (F*(P_*F.transpose())) + Q;
}

Eigen::Vector3d rotateVectorByQuaternion(
    const Eigen::Vector3d& v,
    double q1, double q2, double q3, double q4) {

  Eigen::Matrix<double, 3, 3> C_bn;

  C_bn << (q1*q1-q2*q2-q3*q3+q4*q4), 2*(q1*q2+q3*q4),         2*(q1*q3-q2*q4),
          2*(q1*q2-q3*q4),          -q1*q1+q2*q2-q3*q3+q4*q4, 2*(q2*q3+q1*q4),
          2*(q1*q3+q2*q4),           2*(q2*q3-q1*q4),         -q1*q1-q2*q2+q3*q3+q4*q4; 

  return C_bn * v;
}

void KF::correctionWithMag(double ax, double ay, double az,
  double mx, double my, double mz)
{
  Eigen::Matrix<double, 6, 4> H;  // Measurement model Jacobian
  Eigen::Matrix<double, 4, 6> K;  // Kalman gain
  Eigen::Matrix<double, 6, 6> R;
  Eigen::Matrix<double, 6, 1> Z_est;
  
  double m_norm = sqrt(mx*mx + my*my + mz*mz);
  if(1) {
    mx /= m_norm;
    my /= m_norm;
    mz /= m_norm;
  }
  getReferenceField(mx,my,mz);
  
  R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = Sigma_a_; 
  R.block<3, 3>(3, 3) = Sigma_h_; 

  // Compute the Kalman gain K
  Eigen::Matrix<double, 6, 6> S;
  S = H * P_ * H.transpose() + R;
  K = P_ * H.transpose() * S.inverse();
  
  Eigen::Matrix<double, 6, 1> Error_vec;

  
  //ROS_INFO("MAG_NORM: %f", m_norm);
  //ROS_INFO("H_NORM: %f", h_norm_);
  
  Eigen::Matrix<double, 6, 1>  Z_meas;
  Z_meas << ax, ay, az, mx, my, mz;

  Error_vec = Z_meas - Z_est;

  X_ += K * Error_vec;
  P_ -= K * H * P_;
   
  q1_ = X_(0); q2_ = X_(1); q3_ = X_(2); q4_ = X_(3);
  double a_norm = sqrt (ax*ax + ay*ay + az*az);
  ax /= a_norm;
	ay /= a_norm;
	az /= a_norm;
	q4_ = sqrt((az + 1)*0.5);	
	q1_ = ay/(2*q4_);
	q2_ = -ax/(2*q4_);
	q3_ =0;
	double norm = sqrt(q1_*q1_ + q2_*q2_ + q3_*q3_ + q4_*q4_);
  q1_ /= norm;
  q2_ /= norm;
  q3_ /= norm;
  q4_ /= norm;
	Eigen::Vector3d m;
	m << mx, my, mz;

  Eigen::Vector3d l;

	l << (q1_*q1_ - q2_*q2_ - q3_*q3_ + q4_*q4_)*mx + 2*(q1_*q2_ - q3_*q4_)*my + 2*(q1_*q3_ + q2_*q4_)*mz,
       2*(q1_*q2_ + q3_*q4_)*mx + (-q1_*q1_ + q2_*q2_ - q3_*q3_ + q4_*q4_)*my + 2*(q2_*q3_ - q1_*q4_)*mz,
       2*(q1_*q3_ - q2_*q4_)*mx + 2*(q2_*q3_ + q1_*q4_)*my + (-q1_*q1_ - q2_*q2_ + q3_*q3_ + q4_*q4_)*mz;

 	double q1m, q2m, q3m, q4m;
	double gamma = l(0)*l(0) + l(1)*l(1);	
	double sq_gamma = sqrt(gamma);	
	double den = sqrt(2*(gamma-l(0)*sq_gamma));
	ROS_INFO("sq_gamma, try: %f %f", sq_gamma, (2*(gamma-l(0)*sq_gamma)));
	q1m = 0;
	q2m = 0;
  if (l(1) > 0)
  {
	    q3m = -(sqrt(gamma-l(0)*sq_gamma))/(sqrt(2*gamma));
	    q4m = l(1)/den;
      double qm_norm = sqrt(q3m*q3m + q4m*q4m);
      q3m /= qm_norm;
  	  q4m /= qm_norm;
  }
  else if (l(1) < 0)
  {
      q3m = (sqrt(gamma-l(0)*sq_gamma))/(sqrt(2*gamma));
	    q4m = -l(1)/den;
      double qm_norm = sqrt(q3m*q3m + q4m*q4m);
      q3m /= qm_norm;
  	  q4m /= qm_norm;
   }
   else if (l(1) = 0) 
   { 
		q3m = 1;
  	q4m = 0;
  	}
	//q1_ = q4_*q1m + q1_*q4m + q2_*q3m - q3_*q2m;
	//q2_ = q4_*q2m + q2_*q4m + q3_*q1m - q1_*q3m;
	//q3_ = q4_*q3m + q3_*q3m + q1_*q2m - q2_*q1m;
	//q4_ = q4_*q4m - q1_*q1m - q2_*q2m - q3_*q3m;
  q1_ = 0; q2_ = 0; q3_=q3m; q4_ = q4m; 
    
	ax_ = ax; ay_ = ay; az_ = az;
  ROS_INFO("lx, ly, gamma, den: %f %f %f %f", l(0), l(1), gamma, den);
	ROS_INFO("q3m, q4m: %f %f", q3m, q4m);
  //std::cout << "Z_est:" << std::endl << Z_est << std::endl;
  //std::cout << "Z_meas" << std::endl << Z_meas << std::endl;
  //std::cout << "K:" << std::endl << K << std::endl;
  //std::cout << "P" << P_ << std::endl;
}

void KF::getReferenceField(double mx, double my, double mz)
{
  Eigen::Vector3d h_ref;
    
  h_ref << (q1_*q1_ - q2_*q2_ - q3_*q3_ + q4_*q4_)*mx + 2*(q1_*q2_ - q3_*q4_)*my + 2*(q1_*q3_ + q2_*q4_)*mz,
           2*(q1_*q2_ + q3_*q4_)*mx + (-q1_*q1_ + q2_*q2_ - q3_*q3_ + q4_*q4_)*my + 2*(q2_*q3_ - q1_*q4_)*mz,
           2*(q1_*q3_ - q2_*q4_)*mx + 2*(q2_*q3_ + q1_*q4_)*my + (-q1_*q1_ - q2_*q2_ + q3_*q3_ + q4_*q4_)*mz;

  //h_ref /= h_ref.norm();
  hx_ = sqrt(h_ref(0)*h_ref(0) + h_ref(1)*h_ref(1));
  hy_ = 0;
  hz_ = h_ref(2); 
}
/*
void EKF::correctionWithMag(double ax, double ay, double az,
  double mx, double my, double mz)
{
  Eigen::Vector3d Z_est, Z_meas, Error_vec;
  Eigen::Matrix<double, 3, 3> S;
  Eigen::Matrix<double, 3, 4> H;
  Eigen::Matrix<double, 4, 3> K;
  Eigen::Matrix<double, 4, 4> I4;
  ROS_INFO("q: %f,%f,%f,%f", q1_, q2_,q3_,q4_);

  Z_est << (q1_*q1_ - q2_*q2_ - q3_*q3_ + q4_*q4_)*hx_ + 2*(q1_*q2_ + q3_*q4_)*hy_ + 2*(q1_*q3_ - q2_*q4_)*hz_ ,
           2*(q1_*q2_ - q3_*q4_)*hx_ + (-q1_*q1_ + q2_*q2_ - q3_*q3_ + q4_*q4_)*hy_ + 2*(q2_*q3_ + q1_*q4_)*hz_,
           2*(q1_*q3_ + q2_*q4_)*hx_ + 2*(q2_*q3_ - q1_*q4_)*hy_ + (-q1_*q1_ - q2_*q2_ + q3_*q3_ + q4_*q4_)*hz_;
  std::cout << "Z_est:" << std::endl << Z_est << std::endl;
  H << 2*q1_*hx_+2*q2_*hy_+2*q3_*hz_, -2*q2_*hx_+2*q1_*hy_-2*q4_*hz_, -2*q3_*hx_+2*q4_*hy_+2*q1_*hz_,  2*q4_*hx_+2*q3_*hy_-2*q2_*hz_,  
       2*q2_*hx_-2*q1_*hy_+2*q4_*hz_,  2*q1_*hx_+2*q2_*hy_+2*q3_*hz_, -2*q4_*hx_-2*q3_*hy_+2*q2_*hz_, -2*q3_*hx_+2*q4_*hy_+2*q1_*hz_,    
       2*q3_*hx_-2*q4_*hy_-2*q1_*hz_,  2*q4_*hx_+2*q3_*hy_-2*q2_*hz_,  2*q1_*hx_+2*q2_*hy_+2*q3_*hz_,  2*q2_*hx_-2*q1_*hy_+2*q4_*hz_;
  std::cout << "H" << H << std::endl;
double m_norm = sqrt(mx*mx + my*my + mz*mz);
  if(1) {
    mx /= m_norm;
    my /= m_norm;
    mz /= m_norm;
  }
ROS_INFO("m_norm: %f", m_norm);
  Z_meas << mx, my, mz;
  std::cout << "Z_meas" << std::endl << Z_meas << std::endl;
  Error_vec = Z_meas - Z_est;
  std::cout << "ERROR:" << std::endl << Error_vec << std::endl;
  std::cout << "Ppr" << std::endl << P_ << std::endl;
  std::cout << "Sigma_h" << std::endl << Sigma_h_ << std::endl;
  S = (H*P_*H.transpose()) + Sigma_h_;
  
  Eigen::Matrix<double, 3, 3> S_inv, S_inv_lu;
  Eigen::Matrix<double, 3, 3> I3;
  I3 = Eigen::MatrixXd::Identity(3, 3);
  S_inv = S.inverse();//S.lu().solve(I3);
  S_inv_lu = S.lu().solve(I3);
  std::cout << "Sinv:" << std::endl << Sinv << std::endl;
  std::cout << "Sinv_lu:" << std::endl << S_inv_lu<< std::endl;
  ROS_INFO("q: %f,%f,%f,%f", q1_, q2_,q3_,q4_); 
  K = P_*H.transpose()*S_inv;
 
  std::cout << "K:" << std::endl << K << std::endl;
  X_ += K * Error_vec;
  P_ -=  K * H * P_;
  std::cout << "P" << P_ << std::endl;
  q1_ = X_(0); q2_ = X_(1); q3_ = X_(2); q4_ = X_(3);
  ROS_INFO("q: %f,%f,%f,%f", q1_, q2_,q3_,q4_);
  // Re-normalize quaternion
  double norm = sqrt(q1_*q1_ + q2_*q2_ + q3_*q3_ + q4_*q4_);
  q1_ /= norm;
  q2_ /= norm;
  q3_ /= norm;
  q4_ /= norm;
  ROS_INFO("q: %f,%f,%f,%f", q1_, q2_,q3_,q4_);
  ax_ = ax; ay_ = ay; az_ = az;
   
}
*/
void KF::correctionNoMag(double ax, double ay, double az)
{
  Eigen::Vector3d Z_est, Z_meas, Error_vec;
  Eigen::Matrix<double, 3, 3> S;
  Eigen::Matrix<double, 3, 4> H;
  Eigen::Matrix<double, 4, 3> K;
  Eigen::Matrix<double, 4, 4> I4;

  Z_est << 2*(q1_*q3_ - q2_*q4_)*kGravity,
           2*(q2_*q3_ + q1_*q4_)*kGravity,
           (-q1_*q1_ - q2_*q2_ + q3_*q3_ + q4_*q4_)*kGravity;

  H << kGravity*2*q3_, -kGravity*2*q4_, kGravity*2*q1_, -kGravity*2*q2_, 
       kGravity*2*q4_,  kGravity*2*q3_, kGravity*2*q2_,  kGravity*2*q1_, 
      -kGravity*2*q1_, -kGravity*2*q2_, kGravity*2*q3_,  kGravity*2*q4_;
/*
  double a_norm;
  a_norm = sqrt(ax*ax+ay*ay+az*az);
  ax /= a_norm;
  ay /= a_norm;
  az /= a_norm;
*/
  Z_meas << ax, ay, az;

  Error_vec = Z_meas - Z_est;

  S = (H*(P_*H.transpose())) + Sigma_a_;
  
  K = P_*(H.transpose()*(S.inverse()));

  X_ += K * Error_vec;
  P_ -= K * H * P_;

  q1_ = X_(0); q2_ = X_(1); q3_ = X_(2); q4_ = X_(3);
  
  // Re-normalize quaternion
  double norm = sqrt(q1_*q1_ + q2_*q2_ + q3_*q3_ + q4_*q4_);
  q1_ /= norm;
  q2_ /= norm;
  q3_ /= norm;
  q4_ /= norm;
   
  ax_ = ax; ay_ = ay; az_ = az;

  std::cout << "Z_est:" << std::endl << Z_est << std::endl;
  std::cout << "Z_meas" << std::endl << Z_meas << std::endl;
  std::cout << "K:" << std::endl << K << std::endl;
  std::cout << "P" << P_ << std::endl;
}

void KF::publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  double r_raw ,p_raw ,y_raw;
  tf::Quaternion q_raw;
  tf::quaternionMsgToTF(imu_msg_raw->orientation, q_raw);
  tf::Matrix3x3 M;
  M.setRotation(q_raw);
  M.getRPY(r_raw, p_raw, y_raw);

  double r, p, y;
  tf::Quaternion q(q1_, q2_, q3_, q4_);
  M.setRotation(q);
  M.getRPY(r, p, y);

  tf::Quaternion q_final = tf::createQuaternionFromRPY(r, p, y_raw);
  //tf::Quaternion q(q1_, q2_, q3_, q4_);
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
  transform.setRotation( q_final );
  tf_broadcaster_.sendTransform( tf::StampedTransform( transform,
                   imu_msg_raw->header.stamp,
                   fixed_frame_,
                   imu_frame_ ) );

}

void KF::publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  // create orientation quaternion
  // q4_ is the angle, q1_, q2_, q3_ are the axes
  tf::Quaternion q(q1_, q2_, q3_, q4_);

  // create and publish fitlered IMU message
  boost::shared_ptr<sensor_msgs::Imu> imu_msg = boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);

  imu_msg->header.frame_id = fixed_frame_;
  tf::quaternionTFToMsg(q, imu_msg->orientation);
  imu_KF_publisher_.publish(imu_msg);

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
  yaw_g_msg.data = yaw_g_;

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
  
  bias_gx_publisher_.publish(bias_gx_msg);
  bias_gy_publisher_.publish(bias_gy_msg);  
  bias_gz_publisher_.publish(bias_gz_msg);
    
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 

  double a_x = lin_acc.x;
  double a_y = lin_acc.y;
  double a_z = lin_acc.z;
  double roll_acc, pitch_acc;
  roll_acc  = atan2(a_y, sqrt(a_x*a_x + a_z*a_z));
  pitch_acc = atan2(-a_x, sqrt(a_y*a_y + a_z*a_z));
  std_msgs::Float32 roll_acc_msg;
  std_msgs::Float32 pitch_acc_msg;
  roll_acc_msg.data = roll_acc;
  pitch_acc_msg.data = pitch_acc;
  roll_acc_publisher_.publish(roll_acc_msg);
  pitch_acc_publisher_.publish(pitch_acc_msg);

   // XSens roll and pitch publishing 
   double roll_x, pitch_x, yaw_x;
   tf::Quaternion q_kf; 
   tf::quaternionMsgToTF(imu_msg_raw->orientation, q_kf);
   M.setRotation(q_kf);
   M.getRPY(roll_x, pitch_x, yaw_x);
   std_msgs::Float32 roll_x_msg, pitch_x_msg, yaw_x_msg;
   roll_x_msg.data = roll_x;
   pitch_x_msg.data = pitch_x;
   yaw_x_msg.data = yaw_x;
   roll_xekf_publisher_.publish(roll_x_msg);
   pitch_xekf_publisher_.publish(pitch_x_msg);
   yaw_xekf_publisher_.publish(yaw_x_msg); 
}

/*
 Eigen::Vector3f Z_est_nomag, Z_meas_nomag, Error_vec_nomag, Z_est_nomag1;
    Eigen::Matrix<float, 3, 3> S_nomag;
    Eigen::Matrix<float, 3, 7> H_nomag;
    Eigen::Matrix<float, 7, 3> K_nomag;
     
    Z_est_nomag <<            2*(q1_*q3_-q2_*q4_)*kGravity,// + b_ax,
                              2*(q2_*q3_+q1_*q4_)*kGravity,// + b_ay,
                   (-q1_*q1_-q2_*q2_+q3_*q3_+q4_*q4_)*kGravity;// + b_az;

//Z_est_nomag = C_bn*gravity_vec_ ;//+ bias_vec;
std::cout << "Z" << Z_est_nomag << std::endl;

    H_nomag <<    kGravity*2*q3_, -kGravity*2*q4_, kGravity*2*q1_, -kGravity*2*q2_, 0, 0, 0,
                  kGravity*2*q4_,  kGravity*2*q3_, kGravity*2*q2_,  kGravity*2*q1_, 0, 0, 0,
                 -kGravity*2*q1_, -kGravity*2*q2_, kGravity*2*q3_,  kGravity*2*q4_, 0, 0, 0;
  
  float a_recipNorm;
  a_recipNorm = invSqrt(ax*ax+ay*ay+az*az);
  ax *= a_recipNorm;
  ay *= a_recipNorm;
  az *= a_recipNorm;

    Z_meas_nomag << ax,
                    ay,
                    az;

    Error_vec_nomag = Z_meas_nomag - Z_est_nomag;

    S_nomag = (H_nomag*P*H_nomag.transpose()) + Sigma_a;
  //  std::cout << "S" << S_nomag << std::endl;
    K_nomag = P*H_nomag.transpose()*S_nomag.inverse();
    
    X.noalias() += K_nomag * Error_vec_nomag;
    P.noalias() -= K_nomag*H_nomag*P;
  
//********************** CORRECTION **************************************
C_bn <<  (q1_*q1_-q2_*q2_-q3_*q3_+q4_*q4_),   2*(q1_*q2_+q3_*q4_),                    2*(q1_*q3_-q2_*q4_),
            2*(q1_*q2_-q3_*q4_),            -q1_*q1_+q2_*q2_-q3_*q3_+q4_*q4_,            2*(q2_*q3_+q1_*q4_),
            2*(q1_*q3_+q2_*q4_),             2*(q2_*q3_-q1_*q4_),           -q1_*q1_-q2_*q2_+q3_*q3_+q4_*q4_; 
*/
 // Update State and Covariance  
    

 /*   ROS_INFO("error1: %f,%f,%f,%f,%f,%f", Error_vec(0),Error_vec(1),Error_vec(2), Error_vec(3), Error_vec(4), Error_vec(5));
   
    // Update State and Covariance  
    X.noalias() += K * Error_vec;
    P.noalias() -= K*(H*P);
    
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
*/  


/*
    Eigen::Vector3f Z_est_nomag, Z_meas_nomag, Error_vec_nomag, Z_est_nomag1;
    Eigen::Matrix<float, 3, 3> S_nomag;
    Eigen::Matrix<float, 3, 7> H_nomag;
    Eigen::Matrix<float, 7, 3> K_nomag;
     
    Z_est_nomag <<            2*(q1_*q3_-q2_*q4_)*kGravity,// + b_ax,
                              2*(q2_*q3_+q1_*q4_)*kGravity,// + b_ay,
                   (-q1_*q1_-q2_*q2_+q3_*q3_+q4_*q4_)*kGravity;// + b_az;

//Z_est_nomag = C_bn*gravity_vec_ ;//+ bias_vec;
std::cout << "Z" << Z_est_nomag << std::endl;

    H_nomag <<    kGravity*2*q3_, -kGravity*2*q4_, kGravity*2*q1_, -kGravity*2*q2_, 0, 0, 0,
                  kGravity*2*q4_,  kGravity*2*q3_, kGravity*2*q2_,  kGravity*2*q1_, 0, 0, 0,
                 -kGravity*2*q1_, -kGravity*2*q2_, kGravity*2*q3_,  kGravity*2*q4_, 0, 0, 0;

     

    Z_meas_nomag << ax,
                    ay,
                    az;

    Error_vec_nomag = Z_meas_nomag - Z_est_nomag;

    S_nomag = (H_nomag*P*H_nomag.transpose()) + Sigma_a;
  //  std::cout << "S" << S_nomag << std::endl;
    K_nomag = P*H_nomag.transpose()*S_nomag.inverse();
    
    X.noalias() += K_nomag * Error_vec_nomag;
    P.noalias() -= K_nomag*H_nomag*P;



  
   norm = invSqrt(hx_*hx_+hy_*hy_+hz_*hz_);
    hx_ *= norm;
    hy_ *= norm;
    hz_ *= norm;

  norm = invSqrt(ax*ax+ay*ay+az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    norm = invSqrt(mx*mx+my*my+mz*mz);
    mx *= norm;
    my *= norm;
    mz *= norm;*/

/*  
  m_norm = sqrt(mx*mx + my*my+mz*mz);
   w_norm = sqrt(wx_hut*wx_hut+wy_hut*wy_hut+wz_hut*wz_hut);
   //j = 1-sqrt(q1_*q1_ + q2_*q2_ + q3_*q3_ + q4_*q4_);

   if (w_norm >0)
   {
    s = (sin(w_norm*dt*0.5))/(w_norm*dt*0.5);
    c = cos(w_norm*dt*0.5);
    }
  ROS_INFO("c, s: %f,%f", c, s);
    
ROS_INFO("norm, dt, c, s: %f,%f,%f,%f", w_norm, dt,c,s);
//c + lambda_*j*dt
  q1_ = q1_*c + 0.5*dt*s*( wx_hut*q4_ - wy_hut*q3_ + wz_hut*q2_);
  q2_ = q2_*c + 0.5*dt*s*( wx_hut*q3_ + wy_hut*q4_ - wz_hut*q1_);
  q3_ = q3_*c + 0.5*dt*s*(-wx_hut*q2_ + wy_hut*q1_ + wz_hut*q4_);
  q4_ = q4_*c + 0.5*dt*s*(-wx_hut*q1_ - wy_hut*q2_ - wz_hut*q3_);

*/
  //void EKF::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
//{

//  boost::mutex::scoped_lock(mutex_);

    
//  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
//  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 

//  ros::Time time = imu_msg_raw->header.stamp;
  
/*
 
H <<                                                                       kGravity*2*q3_, -kGravity*2*q4_, kGravity*2*q1_, -kGravity*2*q2_, 0, 0, 0,
                                                                               kGravity*2*q4_,  kGravity*2*q3_, kGravity*2*q2_,  kGravity*2*q1_, 0, 0, 0,
                                                                              -kGravity*4*q1_, -kGravity*4*q2_, 0,  0, 0, 0, 0,
        2*q2_*hy_+2*q3_*hz_, 2*q1_*hy_-2*q4_*hz_-4*q2_*hx_,  2*q4_*hy_+2*q1_*hz_-4*q3_*hx_,  +2*q3_*hy_-2*q2_*hz_,  0, 0, 0,
        2*q2_*hx_+2*q4_*hz_-4*q1_*hx_,  2*q1_*hx_+2*q3_*hz_, -2*q4_*hx_+2*q2_*hz_-4*q3_*hy_, -2*q3_*hx_+2*q1_*hz_,  0, 0, 0,  
        2*q3_*hx_-2*q4_*hy_-4*q1_*hz_,  2*q4_*hx_+2*q3_*hy_-4*q2_*hz_,   2*q1_*hx_+2*q2_*hy_,  2*q2_*hx_-2*q1_*hy_,  0, 0, 0;

  


    H <<                                                                       kGravity*2*q3_, -kGravity*2*q4_, kGravity*2*q1_, -kGravity*2*q2_, 0, 0, 0,
                                                                               kGravity*2*q4_,  kGravity*2*q3_, kGravity*2*q2_,  kGravity*2*q1_, 0, 0, 0,
                                                                              -kGravity*2*q1_, -kGravity*2*q2_, kGravity*2*q3_,  kGravity*2*q4_, 0, 0, 0,
        4*q1_*hx_+2*q2_*hy_+2*q3_*hz_,  2*q1_*hy_+2*q4_*hz_,  -2*q4_*hy_+2*q2_*hz_,  4*q4_*hx_-2*q3_*hy_+2*q2_*hz_,  0, 0, 0,
        2*q2_*hx_+2*q4_*hz_,  2*q1_*hx_+4*q2_*hy_+2*q3_*hz_, 2*q4_*hx_+2*q2_*hz_, 2*q3_*hx_+4*q4_*hy_-2*q1_*hz_,  0, 0, 0,  
        2*q3_*hx_+2*q4_*hy_,  -2*q4_*hx_+2*q3_*hy_,   2*q1_*hx_+2*q2_*hy_+4*q3_*hz_,  -2*q2_*hx_+2*q1_*hy_+4*q4_*hz_,  0, 0, 0;

*/
 /* 
C_bn << -1+2*(q1_*q1_+q4_*q4_),    2*(q1_*q2_-q4_*q3_),    2*(q1_*q3_+q4_*q2_),
          2*(q1_*q2_+q4_*q3_),  1-2*(q1_*q1_+q3_*q3_),    2*(q2_*q3_-q4_*q1_),
          2*(q1_*q3_-q4_*q2_),    2*(q2_*q3_-q4_*q1_),  1-2*(q1_*q1_+q2_*q2_);

C_bn << -1+2*(q1_*q1_+q4_*q4_),    2*(q1_*q2_+ q4_*q3_),    2*(q1_*q3_-q4_*q2_),
          2*(q1_*q2_-q4_*q3_),  1-2*(q1_*q1_+q3_*q3_),    2*(q2_*q3_+q4_*q1_),
          2*(q1_*q3_+q4_*q2_),    2*(q2_*q3_-q4_*q1_),  1-2*(q1_*q1_+q2_*q2_);
 
 

  Eigen::Matrix<float, 6, 6> C;
  C = Eigen::MatrixXf::Zero(6,6);
  C.block<3,3>(0,0)=C_bn;
  C.block<3,3>(3,3)=C_bn;


C_bn <<  (q1_*q1_-q2_*q2_-q3_*q3_+q4_*q4_),   2*(q1_*q2_+q3_*q4_),                    2*(q1_*q3_-q2_*q4_),
            2*(q1_*q2_-q3_*q4_),            -q1_*q1_+q2_*q2_-q3_*q3_+q4_*q4_,            2*(q2_*q3_+q1_*q4_),
            2*(q1_*q3_+q2_*q4_),             2*(q2_*q3_-q1_*q4_),           -q1_*q1_-q2_*q2_+q3_*q3_+q4_*q4_; 

*/
