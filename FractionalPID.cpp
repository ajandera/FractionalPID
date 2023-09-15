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
  eSum=0;
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

void FractionalPIDClass::setESum(float eSum){
  this->eSum=eSum;
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

float FractionalPIDClass::getESum(){
  return eSum;
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
  this->setESum(this->getESum()+err);
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
int FractionalPIDClass::memo(float r, double * c) {
  int m = 0;
  for (int i = 1; i < sizeof(c); i++){
    m = m + c[i] * r;
  }
  return m;
}

// compute u
float FractionalPIDClass::computeU(){
  float pre[] = {};
  float X[]  = {1};
  float X2[]  = {1};
  double bc[]  = {};
  double bd[]  = {};

  /*
  * simulate compud function START
  */ 
  for (int i=1; i <= 3; i++) {
    pre[i] = i;
  }

  for (int j = 2; j < sizeof(pre); j++) {
    X[j] = 1-((-lambda+1)/pre[j]);
  }

  for (int c=2; c <= sizeof(pre); c++) {
    X2[c] = 1-((-lambda+1)/pre[c]);
  }

  // do comprod for X
  for (int i = 1; i <= sizeof(X); i++) {
    bc[i] = X[i-1] * X[i];
  }

  // do comprod for X2
  for (int i = 1; i <= sizeof(X2); i++) {
    bd[i] = X2[i-1] * X2[i];
  }
  /*
  * simulate compud function END
  */

  // calculate the new u
  return (K0 + (1 - K0) * this->getE()) * (Kp * this->getE() + Ti * pow(Ts,lambda) * this->memo(this->getE(), bc) 
              + Td * pow(Ts,-delta) * this->memo(this->getE(), bd));
}

FractionalPIDClass FractionalPID; // Construct instance (define)
