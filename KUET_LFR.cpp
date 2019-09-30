#include "KUET_LFR.h"


// constructor define
Led::Led(int pin) {                                                          // LED constructor
  pinNumber = pin;
  pinMode(pinNumber, OUTPUT);
}

Motor::Motor(int pwm, int forward, int backward) {                           // MOTOR constructor
  pwmPIN = pwm;
  forwardPIN = forward;
  backwardPIN = backward;
  pinMode(pwmPIN, OUTPUT);
  pinMode(forwardPIN, OUTPUT);
  pinMode(backwardPIN, OUTPUT);
}


Robot::Robot(const Motor &m1, const Motor &m2)
  : rm{m1.getPwmPin()},
    lm{m2.getPwmPin()},
    rmf{m1.getForwardPin()},
    lmf{m2.getForwardPin()},
    rmb{m1.getBackwardPin()},
    lmb{m2.getBackwardPin()}
{
  //      pinMode(rm, OUTPUT);
  //      pinMode(lm, OUTPUT);
  //      pinMode(rmf, OUTPUT);
  //      pinMode(lmf, OUTPUT);
  //      pinMode(rmb, OUTPUT);
  //      pinMode(lmb, OUTPUT);
}


//Lfr::Lfr(int rmpwm, int rmForward, int rmBackward, int lmForward, int lmBackward, int lmpwm){                        // Lfr constructor
////  rm = rmpwm;
////  lm = lmpwm;
////  rmf = rmForward;
////  rmb = rmBackward;
////  lmf = lmForward;
////  lmb = lmBackward;
//  pinMode(rm, OUTPUT);
//  pinMode(lm, OUTPUT);
//  pinMode(rmf, OUTPUT);
//  pinMode(lmf, OUTPUT);
//  pinMode(rmb, OUTPUT);
//  pinMode(lmb, OUTPUT);
//}

irSensor::irSensor(String type, uint8_t StartPin, uint8_t EndPin){
  Type = type;
  startPin = StartPin;
  endPin = EndPin;
  if(Type == "digital"){
    for(int i = startPin; i <= endPin; i++){
      pinMode(startPin, OUTPUT);
    }
  }
}




// LED method definitions
void Led::On() {                                                              //led set to on
  digitalWrite(pinNumber, HIGH);
}
void Led::Off() {                                                             //led set to off
  digitalWrite(pinNumber, LOW);
}
void Led::Blink(int onPulse, int offPulse) {                                  // Led blink
  digitalWrite(pinNumber, HIGH);
  delay(onPulse);
  digitalWrite(pinNumber, LOW);
  delay(offPulse);
}

//MOTOR method definition
void Motor::Forward(int ms) {                                                  // motor forward
  analogWrite(pwmPIN, ms);
  digitalWrite(forwardPIN, HIGH);
  digitalWrite(backwardPIN, LOW);
}
void Motor::Backward(int ms) {                                                 //motor backward
  analogWrite(pwmPIN, ms);
  digitalWrite(forwardPIN, LOW);
  digitalWrite(backwardPIN, HIGH);
}
void Motor::Stop() {                                                           //motor stop
  analogWrite(pwmPIN, 0);
  digitalWrite(forwardPIN, LOW);
  digitalWrite(backwardPIN, LOW);
}

//Lfr methods definitions
void Robot::Forward(int ms) {                                                    // Lfr forward move at "ms" speed
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, HIGH);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, HIGH);
  digitalWrite(lmb, LOW);
}
void Robot::Backward(int ms) {                                                  // Lfr backward move at "ms" speed
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, HIGH);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, HIGH);
}
void Robot::Stop() {                                                            //Lfr stops
  analogWrite(rm, 0);
  analogWrite(lm, 0);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, LOW);
}
void Robot::Break(int ms, int Time) {                                            // Breaking system for Lfr, operation runs for "Time" miliseconds
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, HIGH);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, HIGH);
  delay(Time);
}
void Robot::rightTurn(int ms) {                                                // Lfr right turns
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, HIGH);
  digitalWrite(lmf, HIGH);
  digitalWrite(lmb, LOW);
}
void Robot::leftTurn(int ms) {                                                       // Lfr left turns
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, HIGH);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, HIGH);
}

void irSensor::analogValue(uint16_t * av){
  for(uint8_t i = startPin; i <= endPin; i++){
    av[i] = analogRead(i);  }
  
}
void irSensor::digitalValue(uint16_t * dv){
  
}
void irSensor::squaredValue(uint16_t * sv){
  
}
void irSensor::Calibrate(){
  
}
