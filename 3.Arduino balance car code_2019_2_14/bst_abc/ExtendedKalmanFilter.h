#pragma once
class ExtendedKalmanFilter
{
  public:
    /// @brief - Returned filtered angle
    /// @param angle_m := measurement angle (theta)
    /// @param gyro_m  := measuremnt angle velosity (omega) 
    /// @param dt      := time step
    /// @param Q_angle 
    /// @param Q_gyro 
    /// @param R_angle 
    /// @param C_0 
    /// @return 
    float filter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0);
  private:
  
  float stdA = 1.0;
  float varA = stdA*stdA;
  
  float dt   = .005;
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;

  float Xu1 = 0;  // angle at step 0 (initial value for angle)
  float Xu2 = 0 ; // angular velosity at step 0

  float Pu11 = 0; float Pu12 = 0; // P(0|0)
  float Pu21 = 0; float Pu22 = 0; //

  float varAngle = 0.001;
  float varAngVelocity = 0.005;
};