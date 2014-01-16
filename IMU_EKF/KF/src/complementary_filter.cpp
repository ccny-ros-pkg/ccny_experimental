#include "KF/complementary_filter.h"

#include <cmath>

namespace imu_tools {

ComplementaryFilter::ComplementaryFilter() :
    gain_(0.1)
{

}

ComplementaryFilter::~ComplementaryFilter() { }

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

void ComplementaryFilter::initialize(double q0, double q1, double q2, double q3) 
{
  // Set the state to inverse (state is fixed wrt body).
  invertQuaternion(q0, q1, q2, q3, q0_, q1_, q2_, q3_);
}

void ComplementaryFilter::update(double ax, double ay, double az, 
                                 double wx, double wy, double wz,
                                 double dt)
{
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
  makeContinuous(q0_pred, q1_pred, q2_pred, q3_pred,
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
  makeContinuous(q0_pred, q1_pred, q2_pred, q3_pred,
                 q0_meas, q1_meas, q2_meas, q3_meas);

  // Fuse prediction and correction.
  filter(q0_pred, q1_pred, q2_pred, q3_pred,
         q0_meas, q1_meas, q2_meas, q3_meas);
}

void ComplementaryFilter::getPrediction(
    double wx, double wy, double wz, double dt, 
    double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred) const
{
  q0_pred = q0_ + 0.5*dt*( wx*q1_ + wy*q2_ + wz*q3_);
  q1_pred = q1_ + 0.5*dt*(-wx*q0_ - wy*q3_ + wz*q2_);
  q2_pred = q2_ + 0.5*dt*( wx*q3_ - wy*q0_ - wz*q1_);
  q3_pred = q3_ + 0.5*dt*(-wx*q2_ + wy*q1_ - wz*q0_);
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

void ComplementaryFilter::makeContinuous(
    double p0, double p1, double p2, double p3,
    double& q0, double& q1, double& q2, double& q3) const
{
  double delta = p0*q0 + p1*q1 + p2*q2 + p3*q3;
  if (delta < 0)
  {
    q0 *= -1;
    q1 *= -1;
    q2 *= -1;
    q3 *= -1;
  }  
}

void ComplementaryFilter::filter(
    double q0_pred, double q1_pred, double q2_pred, double q3_pred,
    double q0_meas, double q1_meas, double q2_meas, double q3_meas)
{
  // Complementary filter.
  q0_ = (1.0 - gain_) * q0_pred + gain_ * q0_meas;
  q1_ = (1.0 - gain_) * q1_pred + gain_ * q1_meas;
  q2_ = (1.0 - gain_) * q2_pred + gain_ * q2_meas;
  q3_ = (1.0 - gain_) * q3_pred + gain_ * q3_meas;

  // Re-normalize state.
  normalizeQuaternion(q0_, q1_, q2_, q3_);
}

void ComplementaryFilter::getOrientation(
    double& q0, double& q1, double& q2, double& q3) const
{
  // Return the inverse of the state (state is fixed wrt body).
  invertQuaternion(q0_, q1_, q2_, q3_, q0, q1, q2, q3);
}

void ComplementaryFilter::normalizeQuaternion(
    double& q0, double& q1, double& q2, double& q3) const
{
  double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm;  
  q1 /= norm;
  q2 /= norm;
  q3 /= norm;
}

void ComplementaryFilter::invertQuaternion(
    double q0, double q1, double q2, double q3,
    double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv) const
{
  // Assumes quaternion is normalized.
  q0_inv = q0;
  q1_inv = -q1;
  q2_inv = -q2;
  q3_inv = -q3;
}

}  // namespace imu_tools
