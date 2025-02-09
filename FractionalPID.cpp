#include "FractionalPID.h"

FractionalPIDClass::FractionalPIDClass() {
  Kp=1.0;
  Ki=1.0;
  Kd=1.0;
  Ti=1.0;
  Td=1.0;
  Ts=1.0;
  lambda=1;
  delta=1;
  K0=0.0;
  u=0;
  e=0;
  k=1;
  for (int i = 0; i < 100; i++) eHistory[i] = 0;
}

void FractionalPIDClass::setKp(float Kp){
  this->Kp=Kp;
}

void FractionalPIDClass::setKi(float Ki){
  this->Ki=Ki;
  Ti=Kp/Ki;
}

void FractionalPIDClass::setKd(float Kd){
  this->Kd=Kd;
  Td=Kd/Kp;
}

void FractionalPIDClass::setTi(float Ti){
  this->Ti=Ti;
  Ki=Kp/Ti;
}

void FractionalPIDClass::setTd(float Td){
  this->Td=Td;
  Kd=Kp*Td;
}

void FractionalPIDClass::setTs(float Ts){
  this->Ts=Ts;
}

void FractionalPIDClass::setLambda(float lambda){
  this->lambda=lambda;
}

void FractionalPIDClass::setDelta(float delta){
  this->delta=delta;
}

void FractionalPIDClass::setK0(float K0){
  this->K0=K0;
}

void FractionalPIDClass::setE(float e){
  this->e=e;
}

void FractionalPIDClass::setEHistory(float err){
  // Shift error history for fractional calculations
  for (int i = 99; i > 0; i--) eHistory[i] = eHistory[i - 1];
  // Store current error
  eHistory[0] = err;
}

void FractionalPIDClass::setU(float u){
  this->u=u;
}

float FractionalPIDClass::getKp(){
  return Kp;
}

float FractionalPIDClass::getKi(){
  return Ki;
}

float FractionalPIDClass::getKd(){
  return Kd;
}

float FractionalPIDClass::getTi(){
  return Ti;
}

float FractionalPIDClass::getTd(){
  return Td;
}

float FractionalPIDClass::getTs(){
  return Td;
}

float FractionalPIDClass::getLambda(){
  return lambda;
}

float FractionalPIDClass::getDelta(){
  return delta;
}

float FractionalPIDClass::getK0(){
  return K0;
}

float FractionalPIDClass::getE(){
  return e;
}

float FractionalPIDClass::getEHistory(){
  return eHistory;
}

float FractionalPIDClass::getU(){
  return u;
}


float FractionalPIDClass::constrainFloat(float x, float min_x, float max_x){
  if (x<=min_x)
    return min_x;
  else if (x>=max_x)
    return max_x;  
  return x;
}

void FractionalPIDClass::loadVariables(float err){
  this->setE(err);
  this->setEHistory(err);
}

float FractionalPIDClass::compute(float err){    
  loadVariables(err);
  setU(computeU());
  return getU();  
}

float FractionalPIDClass::compute(float err,float saturationMin,float saturationMax) {
  loadVariables(err);   
  setU(constrainFloat(computeU(),saturationMin,saturationMax));
  return getU();
}

// Implementing memory mechanism
int FractionalPIDClass::memo(float r, double * c, int k) {
  float temp = 0;
  for (int j = 1; j < k; j++) {  
      temp += c[j] * r[k - j];  
  }
  return temp;
}

// compute u
float FractionalPIDClass::computeU(int k){
  // Arrays for fractional coefficients
  float bc[k], bd[k];
    
  // Compute fractional coefficients (bc and bd)
  bc[0] = 1;
  bd[0] = 1;
  for (int i = 1; i < k; i++) {
      bc[i] = bc[i - 1] * (1 - (-lambda + 1) / (i + 1));
      bd[i] = bd[i - 1] * (1 - (delta + 1) / (i + 1));
  }

  // Compute control output
    float nonlinear = (K0 + (1 - K0) * abs(this->getE()));
    float proportional = Kp * e;
    float integral = Ti * pow(Ts / 1000.0, lambda) * memo(this->getEHistory(), bc, k);
    float derivative = Td * pow(Ts / 1000.0, -delta) * memo(this->getEHistory(), bd, k);

    return nonlinear * (proportional + integral + derivative);
}

FractionalPIDClass FractionalPID; // Construct instance (define)
