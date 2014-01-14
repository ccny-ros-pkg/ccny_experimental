
#include "KF/kf.h"
#include <iostream>

KF::KF(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  initialized_filter_(false)
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
 
  roll_acc_publisher_    = nh_.advertise<std_msgs::Float32>("roll_acc", queue_size);
  pitch_acc_publisher_   = nh_.advertise<std_msgs::Float32>("pitch_acc", queue_size);
  yaw_m_publisher_   = nh_.advertise<std_msgs::Float32>("yaw_m", queue_size);

  q0_publisher_   = nh_.advertise<std_msgs::Float32>("q0", queue_size);  
  q1_publisher_   = nh_.advertise<std_msgs::Float32>("q1", queue_size);
  q2_publisher_   = nh_.advertise<std_msgs::Float32>("q2", queue_size);
  q3_publisher_   = nh_.advertise<std_msgs::Float32>("q3", queue_size);
  

  // **** register subscribers


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
   
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("imu_frame", imu_frame_))
   imu_frame_ = "imu";
   
}


void KF::imuMagCallback(
  const ImuMsg::ConstPtr& imu_msg_raw,
  const MagMsg::ConstPtr& mag_msg)
{
  boost::mutex::scoped_lock(mutex_);
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& mag_fld = mag_msg->vector;
  
  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;
  
  
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
	  double yaw = atan2( (-my*cos(roll) + mz*sin(roll) ) , 
        (mx*cos(pitch) + my*sin(pitch)*sin(roll) + mz*sin(pitch)*cos(roll)) );
    tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    
    q1_prev_ = 0;
    q2_prev_ = 0;
    q3_prev_ = 0;
    q0_prev_ = 1;
    initialized_filter_ = true; 
  }
  else
  {
    
   if (isnan(mx) || isnan(my) || isnan(mz))
    {
      ROS_INFO("MAG NAN"); 
      
    }
    else 
    {
      normalizeVector(ax, ay, az);
      //normalizeVector(mx, my, mz);
      //acceleration needs to be normalized in order to find the 
      //relative quaternion, magnetic field does not have to!
      double q0_meas, q1_meas, q2_meas, q3_meas;
      getOrientation(ax, ay, az, mx, my, mz, q0_meas, q1_meas, q2_meas, q3_meas);
      q0_ = q0_meas;      
      q1_ = q1_meas;
      q2_ = q2_meas;
      q3_ = q3_meas;
      normalizeQuaternion(q0_, q1_, q2_, q3_ );
    }
    ax_ = ax; ay_ = ay; az_ = az;
    mx_ = mx; my_ = my; mz_ = mz;
    publishFilteredMsg(imu_msg_raw);
    publishTransform(imu_msg_raw);
  }
}


void KF::getOrientation(double ax, double ay, double az, 
                        double mx, double my, double mz,  
                        double& q0, double& q1, double& q2, double& q3)
{
  // q_acc is the quaternion obtained from the acceleration vector representing 
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
  double q0_acc, q1_acc, q2_acc;
    
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
  // l = R(q_acc)^-1 l
  double lx = (q0_acc*q0_acc + q1_acc*q1_acc - q2_acc*q2_acc)*mx + 
      2.0 * (q1_acc*q2_acc)*my - 2.0 * (q0_acc*q2_acc)*mz;
  double ly = 2.0 * (q1_acc*q2_acc)*mx + (q0_acc*q0_acc - q1_acc*q1_acc + 
      q2_acc*q2_acc)*my + 2.0 * (q0_acc*q1_acc)*mz;
  
  // q_mag is the quaternion that rotates the Global frame (North West Up) into
  // the intermediary frame.
  // q1_mag and q2_mag are defined as 0.
	double gamma = lx*lx + ly*ly;	
	double beta = sqrt(gamma + lx*sqrt(gamma));
  double q0_mag = beta / (sqrt(2.0 * gamma));  
  double q3_mag = ly / (sqrt(2.0) * beta); 
  
  // The quaternion multiplication between q_acc and q_mag represents the 
  // quaternion, orientation of the Global frame wrt the local frame.  
  // q = q_acc times q_mag  
  q0 = q0_acc*q0_mag;     
  q1 = q1_acc*q0_mag + q2_acc*q3_mag;
  q2 = q2_acc*q0_mag - q1_acc*q3_mag;
  q3 = q0_acc*q3_mag;

  // It should already be normalized here, up to numerical errors.
  normalizeQuaternion(q0_mag, q1_mag, q2_mag, q3_mag);
}
/*
void KF::checkSolutions(double ax, double ay, double az, double& q0_acc,
    double& q1_acc, double& q2_acc, double& q3_acc)
{
  double p0, p1, p2, p3;
  double q0, q1, q2, q3;  
  //first solution
  p4 = sqrt((az + 1)*0.5);	
  p1 = ay/(2*p4);  
  p2 = -ax/(2*p4);
  p3 = 0;
  //second solution 
  q4 = -sqrt((az + 1)*0.5);	
  q1 = ay/(2*q4);  
  q2 = -ax/(2*q4);
  q3 = 0;
  double dp4, dq4;
  dp4 = computeDeltaQuaternion(p1, p2, p3, p4);
  dq4 = computeDeltaQuaternion(q1, q2, q3, q4);
  ROS_INFO("deltap4, deltaq4: %f, %f", dp4, dq4);
  if ((dp4 >= 0) && (dq4 < 0))
  {
    q1_acc = p1; 
    q2_acc = p2;
    q3_acc = p3;
    q4_acc = p4;
  } 
  else if ((dq4 >=0) && (dp4 < 0))
  {
    q1_acc = q1; 
    q2_acc = q2;
    q3_acc = q3;
    q4_acc = q4;
  }
  else 
  {  
    ROS_ERROR("SOMETHING WRONG??");
  }
}

double KF::computeDeltaQuaternion(double q1, double q2, double q3, double q4)
{
  return (q1*q1_prev_ + q2*q2_prev_ + q4*q4_prev_);
}
*/
void KF::quaternionMultiplication(double p0, double p1, double p2, double p3, 
    double q0, double q1, double q2, double q3, 
    double& r0, double& r1, double& r2, double& r3)
{
  r0 = p0*q0 - p1*q1 - p2*q2 - p3*q3;  
  r1 = p0*q1 + p1*q0 + p2*q3 - p3*q2;
  r2 = p0*q2 - p1*q3 + p2*q0 + p3*q1;
  r3 = p0*q3 + p1*q2 - p2*q1 + p3*q0;
}

void KF::normalizeQuaternion(double& q0, double& q1, double& q2, double& q3)
{
  double q_norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (q_norm == 0.0)
    ROS_ERROR("QUATERNION NORM ZERO") ; 
  q0 /= q_norm;  
  q1 /= q_norm;
  q2 /= q_norm;
  q3 /= q_norm;
}

void KF::normalizeVector(double& x, double& y, double& z)
{
  double norm = sqrt(x*x + y*y + z*z);
  if (norm == 0.0)
    ROS_ERROR("VECTOR NORM ZERO") ; 
  x /= norm;
  y /= norm;
  z /= norm;
}

void KF::invertQuaternion(
    double q0, double q1, double q2, double q3,
    double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv) 
{
  // Assumes quaternion is normalized.
  q0_inv = q0;
  q1_inv = -q1;
  q2_inv = -q2;
  q3_inv = -q3;
}

tf::Quaternion KF::hamiltonToTFQuaternion(
    double q0, double q1, double q2, double q3)
{
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However, 
  // the ROS quternion is in the form [x, y, z, w], with w as the scalar.
  return tf::Quaternion(q1, q2, q3, q0);
}

void KF::publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  // The state is global wrt local frame. Invert to have local wrt global.
  double q0_inv, q1_inv, q2_inv, q3_inv;
  invertQuaternion(q0_, q1_, q2_, q3_, q0_inv, q1_inv, q2_inv, q3_inv);

  // Convert to ROS quaternion.
  tf::Quaternion q = hamiltonToTFQuaternion(q0_inv, q1_inv, q2_inv, q3_inv);

  tf::Transform transform;
  transform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
  transform.setRotation( q );
  tf_broadcaster_.sendTransform( tf::StampedTransform( transform,
                   imu_msg_raw->header.stamp,
                   fixed_frame_,
                   imu_frame_ ) );
}

void KF::publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  // The state is global wrt local frame. Invert to have local wrt global.
  double q0_inv, q1_inv, q2_inv, q3_inv;
  invertQuaternion(q0_, q1_, q2_, q3_, q0_inv, q1_inv, q2_inv, q3_inv);

  // Convert to ROS quaternion.
  tf::Quaternion q = hamiltonToTFQuaternion(q0_inv, q1_inv, q2_inv, q3_inv);

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
  std_msgs::Float32 yaw_m_msg;

  roll_msg.data = roll;
  pitch_msg.data = pitch;
  yaw_msg.data = yaw;
  
  double rolla  = atan2(ay_, sqrt(ax_*ax_ + az_*az_));
  double pitcha = atan2(-ax_, sqrt(ay_*ay_ + az_*az_));
  yaw_m =atan2( (-my_*cos(rolla) + mz_*sin(rolla) ) , 
      (mx_*cos(pitcha) + my_*sin(pitcha)*sin(rolla) + mz_*sin(pitcha)*cos(rolla)));
  //yaw_m =atan2( (mx_*sin(roll)*sin(pitch) + my_*cos(roll) - mz_*sin(roll)*sin(pitch)),(mx_*cos(pitch)+ mz_*sin(pitch)) );
  yaw_m_msg.data = yaw_m;

  roll_ekf_publisher_.publish(roll_msg);
  pitch_ekf_publisher_.publish(pitch_msg);
  yaw_ekf_publisher_.publish(yaw_msg);
  yaw_m_publisher_.publish(yaw_m_msg);  
  
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

  std_msgs::Float32 q1_msg;
  std_msgs::Float32 q2_msg;
  std_msgs::Float32 q3_msg;
  std_msgs::Float32 q0_msg;
  q0_msg.data = q0_;
  q1_msg.data = q1_;
  q2_msg.data = q2_;
  q3_msg.data = q3_;
  q1_publisher_.publish(q1_msg);
  q2_publisher_.publish(q2_msg);
  q3_publisher_.publish(q3_msg);
  q0_publisher_.publish(q0_msg);

  roll_msg.data = roll;
  pitch_msg.data = pitch;
  yaw_msg.data = yaw;

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


