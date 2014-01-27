#include "imu_complementary_filter/complementary_filter.h"

#include <cstdio>
#include <cmath>

namespace imu_tools {

ComplementaryFilter::ComplementaryFilter() :
    gain_(0.1),
    bias_alpha_(0.01),
    do_bias_estimation_(true),
    initialized_(false),
    steady_state_(false),
    q0_(1), q1_(0), q2_(0), q3_(0),
    wx_prev_(0), wy_prev_(0), wz_prev_(0),
    wx_bias_(0), wy_bias_(0), wz_bias_(0) { }

ComplementaryFilter::~ComplementaryFilter() { }

void ComplementaryFilter::setDoBiasEstimation(bool do_bias_estimation)
{
  do_bias_estimation_ = do_bias_estimation;
}

bool ComplementaryFilter::getDoBiasEstimation() const
{
  return do_bias_estimation_;
}

bool ComplementaryFilter::setGain(double gain)
{
  if (gain >= 0 && gain <= 1.0)
  {
    gain_ = gain;
    return true;
  }
  else
    return false;
}

double ComplementaryFilter::getGain() const 
{
  return gain_;
}

bool ComplementaryFilter::getSteadyState() const 
{
  return steady_state_;
}

bool ComplementaryFilter::setBiasAlpha(double bias_alpha)
{
  if (bias_alpha >= 0 && bias_alpha <= 1.0)
  {
    bias_alpha_ = bias_alpha;
    return true;
  }
  else
    return false;
}

double ComplementaryFilter::getBiasAlpha() const 
{
  return bias_alpha_;
}

void ComplementaryFilter::setOrientation(
    double q0, double q1, double q2, double q3) 
{
  // Set the state to inverse (state is fixed wrt body).
  invertQuaternion(q0, q1, q2, q3, q0_, q1_, q2_, q3_);
}


double ComplementaryFilter::getAngularVelocityBiasX() const
{
  return wx_bias_;
}

double ComplementaryFilter::getAngularVelocityBiasY() const
{
  return wy_bias_;
}

double ComplementaryFilter::getAngularVelocityBiasZ() const
{
  return wz_bias_;
}

void ComplementaryFilter::update(double ax, double ay, double az, 
                                 double wx, double wy, double wz,
                                 double dt)
{
  // Bias estimation
  if (do_bias_estimation_)
    updateBiases(ax, ay, az, wx, wy, wz);

  // Prediction.
  double q0_pred, q1_pred, q2_pred, q3_pred;
  getPrediction(wx, wy, wz, dt,
                q0_pred, q1_pred, q2_pred, q3_pred);    

  // "Fake" magnetic reading - according to the predicted state.
  double fx = q0_pred*q0_pred + q1_pred*q1_pred - q2_pred*q2_pred - q3_pred*q3_pred;
  double fy = 2.0*(q1_pred*q2_pred + q0_pred*q3_pred);
  double fz = 2.0*(q1_pred*q3_pred - q0_pred*q2_pred);

  // Measurement.
  double q0_meas, q1_meas, q2_meas, q3_meas;   
  getMeasurement(ax, ay, az, 
                 fx, fy, fz, 
                 q0_meas, q1_meas, q2_meas, q3_meas);

  // Make measurement continuous wrt the prediction.
  makeQuaternionContinuous(q0_pred, q1_pred, q2_pred, q3_pred,
                           q0_meas, q1_meas, q2_meas, q3_meas);

  // Fuse prediction and correction.
  filter(q0_pred, q1_pred, q2_pred, q3_pred,
         q0_meas, q1_meas, q2_meas, q3_meas);
}

void ComplementaryFilter::update(double ax, double ay, double az, 
                                 double wx, double wy, double wz,
                                 double mx, double my, double mz,
                                 double dt)
{
  // Bias estimation
  if (do_bias_estimation_)
    updateBiases(ax, ay, az, wx, wy, wz);

  // Prediction.
  double q0_pred, q1_pred, q2_pred, q3_pred;
  getPrediction(wx, wy, wz, dt,
                q0_pred, q1_pred, q2_pred, q3_pred);     

  // Measurement.
  double q0_meas, q1_meas, q2_meas, q3_meas;   
  getMeasurement(ax, ay, az, 
                 mx, my, mz, 
                 q0_meas, q1_meas, q2_meas, q3_meas);

  // Make measurement continuous wrt the prediction.
  makeQuaternionContinuous(q0_pred, q1_pred, q2_pred, q3_pred,
                           q0_meas, q1_meas, q2_meas, q3_meas);

  // Fuse prediction and correction.
  filter(q0_pred, q1_pred, q2_pred, q3_pred,
         q0_meas, q1_meas, q2_meas, q3_meas);
}

bool ComplementaryFilter::checkState(double ax, double ay, double az, 
                                     double wx, double wy, double wz) const
{
  double acc_magnitude = sqrt(ax*ax + ay*ay + az*az);
  if (fabs(acc_magnitude - kGravity) > kAccelerationThreshold)
    return false;

  if (fabs(wx - wx_prev_) > kDeltaAngularVelocityThreshold ||
      fabs(wy - wy_prev_) > kDeltaAngularVelocityThreshold ||
      fabs(wz - wz_prev_) > kDeltaAngularVelocityThreshold)
    return false;

  if (fabs(wx - wx_bias_) > kAngularVelocityThreshold ||
      fabs(wy - wy_bias_) > kAngularVelocityThreshold ||
      fabs(wz - wz_bias_) > kAngularVelocityThreshold)
    return false;

  return true;
}

void ComplementaryFilter::updateBiases(double ax, double ay, double az, 
                                       double wx, double wy, double wz)
{
  steady_state_ = checkState(ax, ay, az, wx, wy, wz);

  if (steady_state_)
  {
    wx_bias_ += bias_alpha_ * (wx - wx_bias_);
    wy_bias_ += bias_alpha_ * (wy - wy_bias_);
    wz_bias_ += bias_alpha_ * (wz - wz_bias_);
  }

  wx_prev_ = wx; 
  wy_prev_ = wy; 
  wz_prev_ = wz;
}

void ComplementaryFilter::getPrediction(
    double wx, double wy, double wz, double dt, 
    double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred) const
{
  double wx_unb = wx - wx_bias_;
  double wy_unb = wy - wy_bias_;
  double wz_unb = wz - wz_bias_;

  q0_pred = q0_ + 0.5*dt*( wx_unb*q1_ + wy_unb*q2_ + wz_unb*q3_);
  q1_pred = q1_ + 0.5*dt*(-wx_unb*q0_ - wy_unb*q3_ + wz_unb*q2_);
  q2_pred = q2_ + 0.5*dt*( wx_unb*q3_ - wy_unb*q0_ - wz_unb*q1_);
  q3_pred = q3_ + 0.5*dt*(-wx_unb*q2_ + wy_unb*q1_ - wz_unb*q0_);
}

void ComplementaryFilter::getMeasurement(
    double ax, double ay, double az, 
    double mx, double my, double mz,  
    double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas)
{
  // q_acc is the quaternion obtained from the acceleration vector representing 
  // the orientation of the Global frame wrt the Local frame with arbitrary yaw
  // (intermediary frame). q3_acc is defined as 0.
  double q0_acc, q1_acc, q2_acc;
    
  // Normalize acceleration vector.
  normalizeVector(ax, ay, az);

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

void ComplementaryFilter::filter(
    double q0_pred, double q1_pred, double q2_pred, double q3_pred,
    double q0_meas, double q1_meas, double q2_meas, double q3_meas)
{
  if (!initialized_) 
  {
    // First time - ignore prediction
    q0_ = q0_meas;
    q1_ = q1_meas;
    q2_ = q2_meas;
    q3_ = q3_meas;
    initialized_ = true;
  }
  else
  {
    /*
    // Complementary filter.
    q0_ = (1.0 - gain_) * q0_pred + gain_ * q0_meas;
    q1_ = (1.0 - gain_) * q1_pred + gain_ * q1_meas;
    q2_ = (1.0 - gain_) * q2_pred + gain_ * q2_meas;
    q3_ = (1.0 - gain_) * q3_pred + gain_ * q3_meas;
    */

    double dot = q0_pred*q0_meas + q1_pred*q1_meas + q2_pred*q2_meas + 
        q3_pred*q3_meas;

		if (dot < 0.95f)
    {
      // Slerp
      printf("SLERP\n");
      double angle = acos(dot);
      double A = sin(angle*(1.0 - gain_));
      double B = sin(angle * gain_);

      q0_ = A * q0_pred + B * q0_meas;
      q1_ = A * q1_pred + B * q1_meas;
      q2_ = A * q2_pred + B * q2_meas;
      q3_ = A * q3_pred + B * q3_meas;
    }
    else
    {
      // Lerp
      q0_ = (1.0 - gain_) * q0_pred + gain_ * q0_meas;
      q1_ = (1.0 - gain_) * q1_pred + gain_ * q1_meas;
      q2_ = (1.0 - gain_) * q2_pred + gain_ * q2_meas;
      q3_ = (1.0 - gain_) * q3_pred + gain_ * q3_meas;
    }
  }

  // Re-normalize state.
  normalizeQuaternion(q0_, q1_, q2_, q3_);
}

void ComplementaryFilter::getOrientation(
    double& q0, double& q1, double& q2, double& q3) const
{
  // Return the inverse of the state (state is fixed wrt body).
  invertQuaternion(q0_, q1_, q2_, q3_, q0, q1, q2, q3);
}

void normalizeVector(double& x, double& y, double& z)
{
  double norm = sqrt(x*x + y*y + z*z);

  x /= norm;
  y /= norm;
  z /= norm;
}

void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3)
{
  double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm;  
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void invertQuaternion(
    double q0, double q1, double q2, double q3,
    double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv)
{
  // Assumes quaternion is normalized.
  q0_inv = q0;
  q1_inv = -q1;
  q2_inv = -q2;
  q3_inv = -q3;
}

void makeQuaternionContinuous(
    double p0, double p1, double p2, double p3,
    double& q0, double& q1, double& q2, double& q3)
{
  // Calculate the scalar component (q0) of (p * q_inv)
  double delta = p0*q0 + p1*q1 + p2*q2 + p3*q3;

  // If the scalar of the delta quaternion is less than zero, use the
  // alternative formulation for q.
  if (delta < 0)
  {
    q0 *= -1;
    q1 *= -1;
    q2 *= -1;
    q3 *= -1;
  }  
}

}  // namespace imu_tools
