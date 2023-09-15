#include "Arduino.h"

#ifndef FractionalPID_H
#define FractionalPID_H

/*
* FractionalPIDClass
* created by Ales Jandera
*/
class FractionalPIDClass {  
  public:
    FractionalPIDClass();
	
    float compute(float err); // compute action without restrictions
    float computeU(); // compute action without restrictions
    int memo(float r, double * c); // compute action without restrictions
    float compute(float err, float saturationMin, float saturationMax); //compute an action within the given boundaries

    // initial setters
    void setKp(float Kp);
    void setKi(float Ki);
    void setKd(float Kd);
    void setTi(float Ti);
    void setTd(float Td);
	  void setTs(float Ts);
    void setLambda(float lambda);
    void setDelta(float delta);
    void setK0(float K0);
    void setE(float e);
    void setESum(float eSum);
    void setU(float u);

    // initial getters
    float getKp();
    float getKi();
    float getKd();
    float getTi();
    float getTd();
    float getTs();
    float getLambda();
    float getDelta();
    float getK0();
    float getE();
    float getESum();
    float getU();
	
  private:
    void loadVariables(float err);   
    float constrainFloat(float x, float min_x, float max_x);

    // define variables
    float Kp;
    float Ki;
    float Kd;
    float Ti;
    float Td;
    float Ts;
    float lambda;
    float delta;    
    float K0; 
    float u;
    float e;
    float eSum;
};
extern FractionalPIDClass FractionalPID;
#endif
