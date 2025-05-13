#include "ExtendedKalmanFilter.h"

float ExtendedKalmanFilter::filter(double tan_of_angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0)
{
    // Predicted state x(k|k-1) = F*x(k-1|k-1)  
    float Xp1  = Xu1 + dt*Xu2;
    float Xp2 =           Xu2;
 
    // Predicted covariance P(k|k-1) = F*P(k-1|k-1)*F' + G*Q*G'
    // 1st term: F*P(k-1|k-1)*F
    // Pp := P(k|k-1)    <-- predicted cov
    // Pu := P(k-1|k-1)  <-- updated cov
    // Q  := varA := stdA*stdA - variance of angular acceleration
    //        ⌈ 0.5*dt*dt ⌉
    //  G :=  ⌊     dt    ⌋
    float Pp11 = Pu11 + (Pu12+Pu21)*dt + Pu22*dt2;    float Pp12 = Pu12 + Pu22*dt;
    float Pp21 = Pu21 + Pu22*dt;                      float Pp22 = Pu22;                       

    // 2nd term: G*Q*G'
    Pp11 += dt4*0.25*varA;   Pp12 += dt3*0.5*varA;
    Pp21 += dt3*0.5*varA;    Pp22 += dt2*varA;                      

    // Innovation
    float y1 = tan_of_angle_m - Xp1;
    float y2 = gyro_m  - Xp2;

    // Measurement eq z = H*x + v, where H = [[dh, 0], [0, 1]]   2x2 matrix
    // Innovation cov Sk = H*Pp*H' + R, where R = E(v*v') = [[varAngle, 0], [0, varAngVelocity]] <-- diagonal matrix with variances of measurement noise
    float dh = 1 + tan_of_angle_m*tan_of_angle_m;
    float invS22 =  Pp11*dh + varAngle;  float invS21 = -Pp12*dh;
    float invS12 = -Pp21*dh;             float invS11 =  Pp22 + varAngVelocity;
    float invDetS = 1.0/(invS22*invS11 - invS12*invS21);
    
    // Computing Gains K = Pp*H'*invS = Pp*invS;
    float K11 = (Pp11 * dh * invS11 + Pp12 *invS21)*invDetS; float K12 = (Pp11 * dh * invS12 + Pp12 *invS22)*invDetS;
    float K21 = (Pp21 * dh * invS11 + Pp22 *invS12)*invDetS; float K22 = (Pp21 * dh * invS12 + Pp22 *invS22)*invDetS;

    // Updated covariance Pu = (I - KH)*Pp = (I-K)*Pp
    Pu11 = (1-K11*dh)*Pp11-K12*Pp21;  Pu12 = (1-K11*dh)*Pp12-K12*Pp22; 
    Pu21 = -K21*dh*Pp11+(1-K22)*Pp21; Pu22 = -K21*dh*Pp12+(1-K22)*Pp22;

    // Updates states;
    Xu1  = Xp1;
    Xu1 += K11*y1 + K12*y2;
    Xu2  = Xp2;
    Xu2 += K21*y1 + K22*y2;

    return Xu1;
}
