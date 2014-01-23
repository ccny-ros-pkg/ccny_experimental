#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_H

namespace imu_tools {

class ComplementaryFilter
{
  public:
    ComplementaryFilter();    
    virtual ~ComplementaryFilter();

    bool setGain(double gain);
    double getGain() const;

    bool setBiasAlpha(double bias_alpha);
    double getBiasAlpha() const;

    // When the filter is in the steady state, bias estimation will occur (if the
    // parameter is enabled).
    bool getSteadyState() const;

    void setDoBiasEstimation(bool do_bias_estimation);
    bool getDoBiasEstimation() const;

    double getAngularVelocityBiasX() const;
    double getAngularVelocityBiasY() const;
    double getAngularVelocityBiasZ() const;

    // Set the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void setOrientation(double q0, double q1, double q2, double q3);

    // Get the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void getOrientation(double& q0, double& q1, double& q2, double& q3) const;

    // Update from accelerometer and gyroscope data.
    // [ax, ay, az]: Normalized gravity vector.
    // [wx, wy, wz]: Angular veloctiy, in rad / s.
    // dt: time delta, in seconds.
    void update(double ax, double ay, double az, 
                double wx, double wy, double wz,
                double dt);

    // Update from accelerometer, gyroscope, and magnetometer data.
    // [ax, ay, az]: Normalized gravity vector.
    // [wx, wy, wz]: Angular veloctiy, in rad / s.
    // [mx, my, mz]: Magnetic field, units irrelevant.
    // dt: time delta, in seconds.
    void update(double ax, double ay, double az, 
                double wx, double wy, double wz,
                double mx, double my, double mz,
                double dt);

  private:

    static const double kGravity= 9.81;
    static const double kAngularVelocityThreshold = 0.2;
    static const double kAccelerationThreshold = 0.1;
    static const double kDeltaAngularVelocityThreshold = 0.01;

    // Gain parameter for the complementary filter, belongs in [0, 1].
    double gain_;

    // Bias estimation gain parameter, belongs in [0, 1].
    double bias_alpha_;

    // Parameter whether to do bias estimation or not.
    bool do_bias_estimation_;

    bool initialized_;
    bool steady_state_;

    // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
    // the orientation of the fixed frame wrt the body frame.
    double q0_, q1_, q2_, q3_;  

    // Bias in angular velocities;
    double wx_prev_, wy_prev_, wz_prev_;

    // Bias in angular velocities;
    double wx_bias_, wy_bias_, wz_bias_;

    void updateBiases(double ax, double ay, double az, 
                      double wx, double wy, double wz);

    bool checkState(double ax, double ay, double az, 
                    double wx, double wy, double wz) const;

    void getPrediction(
        double wx, double wy, double wz, double dt, 
        double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred) const;

    void getMeasurement(
        double ax, double ay, double az, 
        double mx, double my, double mz,  
        double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas);

    void filter(double q0_pred, double q1_pred, double q2_pred, double q3_pred,
                double q0_meas, double q1_meas, double q2_meas, double q3_meas);
};

void normalizeVector(double& x, double& y, double& z);
void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3);
void invertQuaternion(
    double q0, double q1, double q2, double q3,
    double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv);

// Given two quaternions p and q, this function will calculate the arc distance
// between them. If the arc length is greater than 180, then q will be set to 
// -q, so that the arc distance between them is the shorter one.
void makeQuaternionContinuous(double p0, double p1, double p2, double p3,
                              double& q0, double& q1, double& q2, double& q3);

}  // namespace imu

#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_H
