#include "EKF_tg.h"
#include <math.h>

float EKF_tg::filter(double tg_m, double gyro_m)
{
    // Predicted state x(k|k-1) = f(x(k-1|k-1)) - non linear function  
    float Xp1  = Xu1 + (1+Xu1*Xu1)*Xu2*dt;
    float Xp2 =           Xu2;
 
    // Predicted covariance P(k|k-1) = F*P(k-1|k-1)*F' + Q
    // 1st term: F*P(k-1|k-1)*F
    // Pp := P(k|k-1)    <-- predicted cov
    // Pu := P(k-1|k-1)  <-- updated cov
    // Q  := [varA 0; 0 varO] cov 2x2 matrix of the process noise
    float F11 = 1+2*Xu1*Xu2*dt; float F12 = (1+Xu1*Xu1)*dt;
    float F21 = 0;              float F22 = 1; 
    
    float Pp11 = (F11*Pu11 + F12*Pu21)*F11 + (F11*Pu12 + F12*Pu22)*F12; float Pp12 = (F11*Pu12 + F12*Pu22);
    float Pp21 = F11*Pu21 + F12*Pu22;                                   float Pp22 = Pu22;                       

    // 2nd term: Q
    Pp11 += varA;
    Pp22 += varO;                      

    // Innovation
    float y1 = tg_m    - Xp1;
    float y2 = gyro_m  - Xp2;

    // Measurement eq z = H*x + v, where H=Identity 2x2 matrix
    // Innovation cov Sk = H*Pp*H' + R, where R = E(v*v') = [[varAngle, 0], [0, varAngVelocity]] <-- diagonal matrix with variances of measurement noise
    float invS22 =  Pp11 + varAngle;  float invS21 = -Pp12;
    float invS12 = -Pp21;             float invS11 =  Pp22 + varAngVelocity;
    float invDetS = 1.0/(invS22*invS11 - invS12*invS21);
    
    // Computing Gains K = Pp*H'*invS = Pp*invS;
    float K11 = (Pp11 * invS11 + Pp12 *invS21)*invDetS; float K12 = (Pp11 * invS12 + Pp12 *invS22)*invDetS;
    float K21 = (Pp21 * invS11 + Pp22 *invS12)*invDetS; float K22 = (Pp21 * invS12 + Pp22 *invS22)*invDetS;

    // Updated covariance Pu = (I - KH)*Pp = (I-K)*Pp
    Pu11 = (1-K11)*Pp11-K12*Pp21;  Pu12 = (1-K11)*Pp12-K12*Pp22; 
    Pu21 = -K21*Pp11+(1-K22)*Pp21; Pu22 = -K21*Pp12+(1-K22)*Pp22;

    // Updates states;
    Xu1  = Xp1;
    Xu1 += K11*y1 + K12*y2;
    Xu2  = Xp2;
    Xu2 += K21*y1 + K22*y2;

    return atan(Xu1)*57.3;
}
