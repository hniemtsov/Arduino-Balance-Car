#pragma once
class EKF_tg
{
  public:
    /// @brief - Returned filtered angle
    /// @param angle_m := measurement angle (theta)
    /// @param gyro_m  //ignored
    /// @return 
   float EKF_tg::filter(double tg_m, double gyro_m);
  public:
  
  float stdA = 1.0;
  float varA = stdA*stdA;

  float stdO = 1.0;
  float varO = stdO*stdO;
  
  float dt   = .005;
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;

  float Xu1 = 0;  // angle at step 0 (initial value for angle)
  float Xu2 = 0 ; // angular velosity at step 0

  float Pu11 = 0; float Pu12 = 0; // P(0|0)
  float Pu21 = 0; float Pu22 = 0; //

  float varAngle = 1;
  float varAngVelocity = 100;
};