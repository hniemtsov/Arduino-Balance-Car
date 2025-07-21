/*
BalanceCar.h BalanceCar.cpp - Library for BST-Balance car code.
Created by SKY ZHU&ROOMS LUO, OCTOBER 2, 2016.
JUST FOR THE Company of Technology of yahboom.
In order to  avoid Infringement Act,this core is not for the commerce except being authorized by the writer.
*/



#ifndef BalanceCar_h
#define BalanceCar_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


class BalanceCar
{
public:
void calc_pwms(double& pwm1, double& pwm2, double speedoutput, float rotationoutput);
  double speedpiout(double kps,double kis,double kds,int f,int b,double p0);
  float turnspin(int turnleftflag,int turnrightflag,int spinleftflag,int spinrightflag,double kpturn,double kdturn,float Gyroz);
  
  void pwma(double& pwm1,double& pwm2,float angle,float angle6,int turnleftflag,int turnrightflag,int spinleftflag,int spinrightflag, int f,int b,float accelz,int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB);

	long pulseright = 0;
	long pulseleft = 0;
	int posture=0; // gnem: not used
	int stopl = 0;
	int stopr = 0;
	double angleoutput=0, pwm1 = 0, pwm2 = 0;
	long sumam;
	long count_prev = 0;

#define WINDOW_SIZE 8

	float buffer[WINDOW_SIZE] = { 0,0,0,0,0,0,0,0 };  // Circular buffer
	uint8_t index = 0;          // Index of the oldest sample
	float sum = 0;              // Rolling sum
	float speeds_filter;
	float speeds;
	float positions;
	float new_sample;
private:
	float speeds_filterold = 0;//ËÙ¶ÈÂË²¨
	
	int turnmax = 0;                                    //Ðý×ªÊä³ö·ùÖµ
	int turnmin = 0;                                  //Ðý×ªÊä³ö·ùÖµ
	float turnout = 0;
	int flag1 = 0;
	int flag2 = 0;
	int flag3 = 0;
	int flag4 = 0;

};
#endif
//
// END OF FILE
//