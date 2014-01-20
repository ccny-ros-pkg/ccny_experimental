#include "imu_complementary_filter/complementary_filter_ros.h"

#include <std_msgs/Float64.h>

namespace imu_tools {

ComplementaryFilterROS::ComplementaryFilterROS(
    const ros::NodeHandle& nh, 
    const ros::NodeHandle& nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_filter_(false)
{
  ROS_INFO("Starting KF");
  initializeParams();
  
  int queue_size = 5;

  // Register publishers:
  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu/data", queue_size);
  roll_publisher_ = nh_.advertise<std_msgs::Float64>("imu/roll", queue_size);
  pitch_publisher_ = nh_.advertise<std_msgs::Float64>("imu/pitch", queue_size);
  yaw_publisher_ = nh_.advertise<std_msgs::Float64>("imu/yaw", queue_size);
   
  // Register IMU raw data subscriber.
  imu_subscriber_.reset(new ImuSubscriber(nh_, "imu/data_raw", queue_size));

  // Register magnetic data subscriber.
  if (use_mag_)
  {
    mag_subscriber_.reset(new MagSubscriber(nh_, "imu/mag", queue_size));

    sync_.reset(new Synchronizer(
        SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
    sync_->registerCallback(
        boost::bind(&ComplementaryFilterROS::imuMagCallback, this, _1, _2));
  }
  else
  {
    imu_subscriber_->registerCallback(
        &ComplementaryFilterROS::imuCallback, this);
  } 
}

ComplementaryFilterROS::~ComplementaryFilterROS()
{
  ROS_INFO("Destroying ComplementaryFilterROS");
}

void ComplementaryFilterROS::initializeParams()
{
  double gain;
  bool do_bias_estimation;
  double bias_alpha;

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("use_mag", use_mag_))
    use_mag_ = false;
  if (!nh_private_.getParam ("gain", gain))
    gain = 0.1;
  if (!nh_private_.getParam ("do_bias_estimation", do_bias_estimation))
    do_bias_estimation = true;
  if (!nh_private_.getParam ("bias_alpha", bias_alpha))
    bias_alpha = 0.01;

  filter_.setDoBiasEstimation(do_bias_estimation);

  if(!filter_.setGain(gain))
    ROS_WARN("Invalid gain passed to ComplementaryFilter.");
  if(!filter_.setBiasAlpha(bias_alpha))
    ROS_WARN("Invalid bias_alpha passed to ComplementaryFilter.");
}

void ComplementaryFilterROS::imuCallback(const ImuMsg::ConstPtr& imu_msg_raw)
{
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const ros::Time& time = imu_msg_raw->header.stamp;
    
  double ax = lin_acc.x;
  double ay = lin_acc.y;
  double az = lin_acc.z;
  double wx = ang_vel.x;
  double wy = ang_vel.y;
  double wz = ang_vel.z;

  if (!initialized_filter_)
  {   
    time_prev_ = time;
    initialized_filter_ = true;
    return; 
  }

  // Calculate dt.
  double dt = (time - time_prev_).toSec();
  time_prev_ = time;

  // Update the filter.    
  filter_.update(ax, ay, az, wx, wy, wz, dt);

  // Publish state.     
  publish(imu_msg_raw);
}

void ComplementaryFilterROS::imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                                            const MagMsg::ConstPtr& mag_msg)
{
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& mag_fld = mag_msg->vector;
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const ros::Time& time = imu_msg_raw->header.stamp;
    
  double ax = lin_acc.x;
  double ay = lin_acc.y;
  double az = lin_acc.z;
  double wx = ang_vel.x;
  double wy = ang_vel.y;
  double wz = ang_vel.z;
  double mx = mag_fld.x;
  double my = mag_fld.y;
  double mz = mag_fld.z;

  if (!initialized_filter_)
  {   
    time_prev_ = time;
    initialized_filter_ = true;
    return; 
  }
 
  // Calculate dt.
  double dt = (time - time_prev_).toSec();
  time_prev_ = time;

  // Update the filter.    
  if (isnan(mx) || isnan(my) || isnan(mz))
    filter_.update(ax, ay, az, wx, wy, wz, dt);
  else 
    filter_.update(ax, ay, az, wx, wy, wz, mx, my, mz, dt);

  // Publish state.     
  publish(imu_msg_raw);
}

tf::Quaternion ComplementaryFilterROS::hamiltonToTFQuaternion(
    double q0, double q1, double q2, double q3) const
{
  // ROS uses the Hamilton quaternion convention (q0 is the scalar). However, 
  // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
  return tf::Quaternion(q1, q2, q3, q0);
}

void ComplementaryFilterROS::publish(
    const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  // Get the orientation:
  double q0, q1, q2, q3;
  filter_.getOrientation(q0, q1, q2, q3);
  tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

  // Create and publish fitlered IMU message
  boost::shared_ptr<sensor_msgs::Imu> imu_msg = 
      boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);
  tf::quaternionTFToMsg(q, imu_msg->orientation);
  imu_publisher_.publish(imu_msg);

  // Create and publish roll, pitch, yaw angles
  double roll, pitch, yaw;
  tf::Matrix3x3 M;
  M.setRotation(q);
  M.getRPY(roll, pitch, yaw);
  std_msgs::Float64 roll_msg;
  std_msgs::Float64 pitch_msg;
  std_msgs::Float64 yaw_msg;

  roll_msg.data = roll;
  pitch_msg.data = pitch;
  yaw_msg.data = yaw;
  
  roll_publisher_.publish(roll_msg);
  pitch_publisher_.publish(pitch_msg);
  yaw_publisher_.publish(yaw_msg);

  // Create and publish the ROS tf.
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
  transform.setRotation( q );
  tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform,
                           imu_msg_raw->header.stamp,
                           fixed_frame_,
                           imu_msg_raw->header.frame_id));
}

}  // namespace imu_tools
