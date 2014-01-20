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

    // Gain parameter for the complementary filter, belongs in [0, 1].
    double gain_;
  
    bool initialized_;

    // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
    // the orientation of the fixed frame wrt the body frame.
    double q0_, q1_, q2_, q3_;  

    void getPrediction(
        double wx, double wy, double wz, double dt, 
        double& q0_pred, double& q1_pred, double& q2_pred, double& q3_pred) const;

    void getMeasurement(
        double ax, double ay, double az, 
        double mx, double my, double mz,  
        double& q0_meas, double& q1_meas, double& q2_meas, double& q3_meas);

    void makeContinuous(double p0, double p1, double p2, double p3,
                        double& q0, double& q1, double& q2, double& q3) const;

    void filter(double q0_pred, double q1_pred, double q2_pred, double q3_pred,
                double q0_meas, double q1_meas, double q2_meas, double q3_meas);

    void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3) const;

    void invertQuaternion(
        double q0, double q1, double q2, double q3,
        double& q0_inv, double& q1_inv, double& q2_inv, double& q3_inv) const;
};

}  // namespace imu

#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_H
