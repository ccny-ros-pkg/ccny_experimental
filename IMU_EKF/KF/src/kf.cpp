
#include "KF/kf.h"
#include <KF/EXTERNAL/eigen3/Eigen/Eigen>
#include <KF/EXTERNAL/eigen3/Eigen/Geometry>
#include <KF/EXTERNAL/eigen3/Eigen/LU>
#include <iostream>

KF::KF(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  initialized_filter_(false),
  initialized_LP_filter_(false),
  q0(1.0), q1(0.0), q2(0.0), q3(0.0)
  
{
  ROS_INFO ("Starting KF");

  // **** get paramters 
  // nothing for now
  
  initializeParams();
  
  int queue = 5;

  imu_EKF_publisher_ = nh_.advertise<sensor_msgs::Imu>(
	    "imu_EKF/data", queue);

  roll_kf_publisher_    = nh.advertise<std_msgs::Float32>("roll_kf", queue);
  pitch_kf_publisher_   = nh.advertise<std_msgs::Float32>("pitch_kf", queue);
  yaw_kf_publisher_     = nh.advertise<std_msgs::Float32>("yaw_kf", queue);
  
  roll_acc_publisher_    = nh.advertise<std_msgs::Float32>("roll_acc", queue);
  pitch_acc_publisher_   = nh.advertise<std_msgs::Float32>("pitch_acc", queue);
 

  // **** register subscribers
  int queue_size = 5;

  imu_subscriber_.reset(new ImuSubscriber(
    nh_, "imu/data_raw", queue_size));
  
  mag_subscriber_.reset(new MagSubscriber(
     nh_, "imu/mag", queue_size));
  sync_.reset(new Synchronizer(
     SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
  sync_->registerCallback(boost::bind(&KF::imuMagCallback, this, _1, _2));
  }


KF::~KF()
{
  ROS_INFO ("Destroying KF");
}

void KF::initializeParams()
{
   //last_time_ = 0.0;
 // initialized_ = false;
  
  if (!nh_private_.getParam ("sigma_g", sigma_g_))
    sigma_g_ = 0.01;
  
  if (!nh_private_.getParam ("sigma_a", sigma_a_))
    sigma_a_ = 0.01;

  if (!nh_private_.getParam ("sigma_m", sigma_m_))
    sigma_m_ = 0.01;
  
  if (!nh_private_.getParam ("constant_dt", constant_dt_))
      constant_dt_ = 0.0;
        
  gravity_vec_ = Eigen::Vector3f(0, 0, gmag_);
  h_ref_vec_  = Eigen::Vector3f(0.193, -0.046, 0.503);
  
}


void KF::imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                         const MagMsg::ConstPtr& mag_msg)   
{

  boost::mutex::scoped_lock(mutex_);
  
  //imu.header.stamp = ros::Time::now();
    
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& mag_fld = mag_msg->vector;
  ros::Time time = imu_msg_raw->header.stamp;
 
  float ax = lin_acc.x;
  float ay = lin_acc.y;
  float az = lin_acc.z;

  float mx = mag_fld.x;
  float my = mag_fld.y;
  float mz = mag_fld.z;  

  if(std::isnan(mag_fld.x) || std::isinf(mag_fld.x) ||
     std::isnan(mag_fld.y) || std::isinf(mag_fld.y) ||
     std::isnan(mag_fld.z) || std::isinf(mag_fld.z) ) 
      {
        mx = mx_prev_ ;
        my = my_prev_ ;
        mz = mz_prev_ ; 
        
      }   
  mx_prev_ = mx;
  my_prev_ = my;
  mz_prev_ = mz ;
  //ROS_INFO("magnetic field: %f,%f,%f", mx , my ,mz);
  
    
  float dt;
  if (!initialized_filter_)
    {
      mx_prev_ = 0;
      my_prev_ = 0;
      mz_prev_ = 0;
        
      if(std::isnan(mag_fld.x) || std::isinf(mag_fld.x) ||
        std::isnan(mag_fld.y) || std::isinf(mag_fld.y) ||
        std::isnan(mag_fld.z) || std::isinf(mag_fld.z) ) 
      {
        mx = 0 ;
        my = 0 ;
        mz = 0 ; 
      }  
      Vector4f q_acc, q_mag, q_meas;
      float lx, ly;   
      getMeasurement(ax, ay, az, mx, my, mz, lx, ly, q_acc, q_mag);
      quaternionMultiplication(q_acc, q_mag, q_meas);
      X_ = q_meas;
                    
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
      
      
      Vector4f q_pred;
      ros::Time t_in, t_out;        
      t_in = ros::Time::now();
      getPrediction(ang_vel.x, ang_vel.y, ang_vel.z, dt, q_pred);
      //ROS_INFO("PREDICTION: %f,%f,%f,%f", q_pred(0), q_pred(1), q_pred(2), q_pred(3)); 
      update(lin_acc.x, lin_acc.y, lin_acc.z, mx, my, mz, q_pred);
      //ROS_INFO("X_: %f,%f,%f,%f", X_(0), X_(1), X_(2), X_(3));
      t_out = ros::Time::now(); 
      float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
      printf("%.6f\n", dt_tot);
  
      //std::cout << dt_tot << std::endl;
      std_msgs::Float32 acc_mag_msg;
      //acc_mag_msg.data = a_magnitude;
      //acc_mag_publisher_.publish(acc_mag_msg);
      publishFilteredMsg(imu_msg_raw);
   
     }
}

void KF::getPrediction(float wx, float wy, float wz, 
                       float dt, Vector4f& q_pred)
{  
   Eigen::Matrix<float, 4, 4> F;  // State-transition matrix   
   Eigen::Matrix<float, 4, 3> Xi;           
   Eigen::Matrix<float, 4, 4> Q;  // process noise covariance
   Eigen::Matrix3f Sigma_g, I3;
   I3 = Eigen::Matrix3f::Identity(); 
     
   q0 = X_(0);
   q1 = X_(1);
   q2 = X_(2);
   q3 = X_(3);
  
   // compute "a priori" state estimate 
   X_(0) = q0 + 0.5 * dt*( wx*q1 + wy*q2 + wz*q3);   
   X_(1) = q1 + 0.5 * dt*(-wx*q0 - wy*q3 + wz*q2);
   X_(2) = q2 + 0.5 * dt*( wx*q3 - wy*q0 - wz*q1);
   X_(3) = q3 + 0.5 * dt*(-wx*q2 + wy*q1 - wz*q0);
   float norm = sqrt(X_(0)*X_(0) + X_(1)*X_(1) + X_(2)*X_(2) + X_(3)*X_(3));
   X_ = X_/norm; 
   q_pred = X_;
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

   //std::cout << "Xi matrix" << std::endl << Xi << std::endl; 
   float dt2 = dt*dt*0.25;
   //ROS_INFO("delta t square: %f", dt2);
   Q.noalias() = dt2 * (Xi *Sigma_g * Xi.transpose());
   //std::cout << "Q_priori matrix" << std::endl << Q << std::endl; 
   P_ = F*(P_*F.transpose()) + Q;
       
}

void KF::getMeasurement(
     float ax, float ay, float az, 
     float mx, float my, float mz, 
     float& lx, float& ly,    
     Vector4f& q_acc,
     Vector4f& q_mag)
{
  float norm_a = sqrt(ax*ax + ay*ay + az*az);
  ax /= norm_a;
  ay /= norm_a;
  az /= norm_a; 

  float norm_m = sqrt(mx*mx + my*my + mz*mz);
  mx /= norm_m;
  my /= norm_m;
  mz /= norm_m;

  if (az >=0)
    {
      q_acc(0) =  sqrt((az + 1) * 0.5);	
      q_acc(1) = -ay/(2.0 * q_acc(0));
      q_acc(2) =  ax/(2.0 * q_acc(0));
      q_acc(3) = 0;
    }
  else 
    {
      float X = sqrt((1 - az) * 0.5);
      q_acc(0) = -ay/(2.0 * X);
      q_acc(1) = X;
      q_acc(2) = 0;
      q_acc(3) = ax/(2.0 * X);
    }  
  
  float norm = sqrt(q_acc(0)*q_acc(0) + q_acc(1)*q_acc(1) + q_acc(2)*q_acc(2) + q_acc(3)*q_acc(3));
  q_acc = q_acc/norm;

  // [lx, ly, lz] is the magnetic field reading, rotated into the intermediary
  // frame by the inverse of q_acc.
  // l = R(q_acc)^-1 m
  lx = (q_acc(0)*q_acc(0) + q_acc(1)*q_acc(1) - q_acc(2)*q_acc(2))*mx + 
      2.0 * (q_acc(1)*q_acc(2))*my - 2.0 * (q_acc(0)*q_acc(2))*mz;
  ly = 2.0 * (q_acc(1)*q_acc(2))*mx + (q_acc(0)*q_acc(0) - q_acc(1)*q_acc(1) + 
      q_acc(2)*q_acc(2))*my + 2.0 * (q_acc(0)*q_acc(1))*mz;
  
 // q_mag is the quaternion that rotates the Global frame (North West Up) into
  // the intermediary frame. q1_mag and q2_mag are defined as 0.
	float gamma = lx*lx + ly*ly;	
	float beta = sqrt(gamma + lx*sqrt(gamma));
  q_mag(0) = beta / (sqrt(2.0 * gamma));  
  q_mag(1) = 0;
  q_mag(2) = 0;
  q_mag(3) = ly / (sqrt(2.0) * beta); 
   
  float norm_mag = sqrt(q_mag(0)*q_mag(0) + q_mag(1)*q_mag(1) + q_mag(2)*q_mag(2) + q_mag(3)*q_mag(3));
  q_mag = q_mag/norm_mag;
  //ROS_INFO("q_mag: %f,%f,%f,%f", q_mag(0), q_mag(1), q_mag(2), q_mag(3));
}

void KF::getMeasCovariance(
     float ax, float ay, float az, 
     float mx, float my, float mz,  
     float lx, float ly,  
     Vector4f q_acc,
     Vector4f q_mag,
     Eigen::Matrix<float, 4, 4>& Q)
{  

  Eigen::Matrix<float, 6, 6> Sigma;  // process noise covariance
  Eigen::Matrix<float, 4, 8> H1;
  Eigen::Matrix<float, 8, 6> H2;  
  Eigen::Matrix<float, 6, 6> H3;    
  Eigen::Matrix<float, 4, 6> J;
  Eigen::Matrix3f I3;
  I3 = Eigen::Matrix3f::Identity(); 
  //N2 =  norm(a,2)^2; 
  float norm_a = sqrt(ax*ax + ay*ay + az*az);
  ax /= norm_a;
  ay /= norm_a;
  az /= norm_a;
  float N2 = 9.81*9.81;
  float norm_m = sqrt(mx*mx + my*my + mz*mz);
  mx /= norm_m;
  my /= norm_m;
  mz /= norm_m;
  float Nm2 = 0.25;
  Sigma = Eigen::MatrixXf::Zero(6,6);
  Sigma.block<3,3>(0,0) = (1/N2) * sigma_a_ * I3; 
  Sigma.block<3,3>(3,3) = (1/Nm2) * sigma_m_ * I3;  
  float gamma = lx*lx + ly*ly;
  float beta = sqrt(ly*ly + lx*(lx + sqrt(gamma)));
  if (az > 0)      
  {  
    H1 << q_mag(0), 0, 0, -q_mag(3), q_acc(0), -q_acc(1), -q_acc(2), 0,
          0, q_mag(0), q_mag(3), 0, q_acc(1), q_acc(0), 0, q_acc(2),
          0, -q_mag(3), q_mag(0), 0, q_acc(2), 0, q_acc(0), -q_acc(1),
          q_mag(3), 0, 0, q_mag(0), 0, -q_acc(2), q_acc(1), q_acc(0);
    
    float k = sqrt(1+az);
    float k2 = k*k;
    float k3 = k*k*k;
    float k4 = k2*k2;
    float gamma32 = sqrt(gamma)*sqrt(gamma)*sqrt(gamma);

    H2 = Eigen::MatrixXf::Zero(8,6);
    
    //H2 << 0, 0, 1/k, 0, 0, 0,
    //      0, -2/k, ay/k3, 0, 0, 0,
    //      2/k, 0, -ax/k3, 0, 0, 0,
    //      0, 0, 0, 0, 0, 0,
    //      0, 0, 0, ly*ly/(beta*gamma), lx*ly/(beta*gamma), 0,
    //      0, 0, 0, 0, 0, 0,
    //      0, 0, 0, 0, 0, 0,
    //     0, 0, 0, -(ly*beta)/gamma32, lx*beta/gamma32, 0;

    H2(0,2) = (1/(2*sqrt(2)))*1/k;
    H2(1,1) = (1/(2*sqrt(2)))*(-2/k);
    H2(1,2) = (1/(2*sqrt(2)))*(ay/k3);
    H2(2,0) = (1/(2*sqrt(2)))*2/k;
    H2(2,2) = (1/(2*sqrt(2)))*(-ax/k3);
    H2(4,3) = (1/(2*sqrt(2)))*ly*ly/(beta*gamma);
    H2(4,4) = (1/(2*sqrt(2)))*lx*ly/(beta*gamma);
    H2(7,3) = -(1/(2*sqrt(2)))*(ly*beta)/gamma32;
    H2(7,4) = (1/(2*sqrt(2)))*lx*beta/gamma32;
  
    //H2 = (1/(2*sqrt(2)))*H2;
 
    H3 << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          mz-(2*ax*mx + ay*my)/k2,    - ax*my/k2, ax*(ax*mx + ay*my)/k4,
                            1-(ax*ax)/k2, -(ax*ay)/k2, ax,
         -ay*mx/k2, mz-(ax*mx + 2*ay*my)/k2, ay*(ax*mx + ay*my)/k4,
                       -(ax*ay)/k2, 1-(ay*ay)/k2, ay,
         -mx, -my,              mz, -ax,                 -ay, az;
  }    
  else
  {
    H1 << q_mag(0), 0, 0, -q_mag(3), q_acc(0), -q_acc(1), 0, -q_acc(3),
          0, q_mag(0), q_mag(3), 0, q_acc(1), q_acc(0),  -q_acc(3), 0,
          0, -q_mag(3), q_mag(0), 0, 0, q_acc(3), q_acc(0), -q_acc(1),
          q_mag(3), 0, 0, q_mag(0), q_acc(3), 0, q_acc(1), q_acc(0);

    float k = sqrt(1-az);
    float k2 = k*k;
    float k3 = k*k*k;
    float gamma32 = sqrt(gamma)*sqrt(gamma)*sqrt(gamma);     
   
    H2 = Eigen::MatrixXf::Zero(8,6);
    //H2 << 0, -2/k, -ay/k3, 0, 0, 0,
    //      0, 0, -1/k, 0, 0, 0,
    //      0, 0, 0, 0, 0, 0,
    //      2/k, 0, ax/k3, 0, 0, 0,
    //      0, 0, 0, ly*ly/(beta*gamma), -lx*ly/(beta*gamma), 0,
    //      0, 0, 0, 0, 0, 0,
    //      0, 0, 0, 0, 0, 0,
    //      0, 0, 0, -(ly*beta)/gamma32, lx*beta/gamma32, 0,
    
    H2(0,1) =  (1/(2*sqrt(2)))*(-2/k);
    H2(0,2) =  (1/(2*sqrt(2)))*(-ay/k3);    
    H2(1,2) = -(1/(2*sqrt(2)))*1/k;
    H2(3,0) =  (1/(2*sqrt(2)))*2/k;
    H2(3,2) =  (1/(2*sqrt(2)))*(ax/k3); 
    H2(4,3) =  (1/(2*sqrt(2)))*ly*ly/(beta*gamma);
    H2(4,4) =  (1/(2*sqrt(2)))*lx*ly/(beta*gamma);
    H2(7,3) = -(1/(2*sqrt(2)))*(ly*beta)/gamma32;
    H2(7,4) =  (1/(2*sqrt(2)))*lx*beta/gamma32;
    
    H3 << 1, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          mz-(2*ax*mx - ay*my)/k2,  ax*my/k2, ax*(-ax*mx+ay*my)/(k2*k2), 1-ax*ax/k2, (ax*ay)/k2, ax,
          -ay*mx/k2, mz-(ax*mx - 2*ay*my)/k2, ay*(-ax*mx + ay*my)/(k2*k2), -(ax*ay)/k2, -1+(ay*ay)/k2, ay,
           mx, -my,  -mz, ax, -ay, -az;
  }                    
  J = H1*H2*H3; 
  //std::cout << "Sigma matrix" << std::endl << Sigma << std::endl;
  Q = J*Sigma*J.transpose();
  
} 

void KF::update(float ax, float ay, float az,
                float mx, float my, float mz, 
                const Vector4f q_pred)
{
    Eigen::Matrix<float, 4, 4> K, Q, S, I4;  // Kalman gain
    Eigen::Matrix3f I3, Sigma_a, R;
    
    I3 = Eigen::Matrix3f::Identity();
    I4 = Eigen::MatrixXf::Identity(4,4);
    
    Vector4f q_acc, q_mag, q_meas;
    
    float lx, ly;
    
    getMeasurement(ax, ay, az, mx, my, mz, lx, ly, q_acc, q_mag);  
    quaternionMultiplication(q_acc, q_mag, q_meas);
    //ROS_INFO("MEASUREMENT: %f,%f,%f,%f", q_meas(0), q_meas(1), q_meas(2), q_meas(3)); 
    getMeasCovariance(ax, ay, az, mx, my, mz, lx, ly, q_acc, q_mag, Q);
    makeContinuos(q_pred, q_meas);
    
    Eigen::Matrix<float, 4, 1>  Error_vec;
    
    
    Error_vec = q_meas - q_pred;
    
    //ROS_INFO("ERROR: %f,%f,%f,%f", Error_vec(0), Error_vec(1), Error_vec(2), Error_vec(3)); 
    //Compute the Kalman gain K  
    
    S = P_ + Q;
    //std::cout << "S matrix" << std::endl << S << std::endl;
     
    Eigen::Matrix<float, 4, 4> S_inv;
    S_inv = S.inverse();    
    //S_inv = S.fullPivLu() .solve(I4);
    
    K = P_ * S_inv;    
    
    // Update State and Covariance  
    X_ += K * Error_vec;
    P_ = (I4 - K)*P_;
    float norm = sqrt(X_(0)*X_(0) + X_(1)*X_(1) + X_(2)*X_(2) + X_(3)*X_(3));
    X_ = X_/norm;
    
}

void KF::makeContinuos(const Vector4f q_pred, Vector4f& q_meas)
{     
  float delta = q_pred(0)*q_meas(0) + q_pred(1)*q_meas(1) +
                           q_pred(2)*q_meas(2) + q_pred(3)*q_meas(3);
  if (delta < 0)
  {
    q_meas = -q_meas;
    ROS_WARN("SWITCH");
  }
}

void KF::quaternionMultiplication(Vector4f p, Vector4f q, Vector4f& q_out)
{
  q_out(0) = p(0)*q(0) - p(1)*q(1) - p(2)*q(2) - p(3)*q(3);
  q_out(1) = p(0)*q(1) + p(1)*q(0) + p(2)*q(3) - p(3)*q(2);
  q_out(2) = p(0)*q(2) - p(1)*q(3) + p(2)*q(0) + p(3)*q(1);
  q_out(3) = p(0)*q(3) + p(2)*q(2) - p(3)*q(1) + p(3)*q(0);
}
void KF::publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
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
  roll_kf_publisher_.publish(roll_msg);
  pitch_kf_publisher_.publish(pitch_msg);
  yaw_kf_publisher_.publish(yaw_msg);

  
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









  

