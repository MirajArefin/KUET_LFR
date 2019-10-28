#include "KUET_LFR.h"

// constructor define
Led::Led(int pin) {                                                          // LED constructor
  pinNumber = pin;
  pinMode(pinNumber, OUTPUT);
}

Motor::Motor(uint8_t pwm, uint8_t forward, uint8_t backward) {                           // MOTOR constructor
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

IrSensor::IrSensor(String type, uint8_t StartPin, uint8_t EndPin, float thresh) {
  Type = type;
  startPin = StartPin;
  endPin = EndPin;
  threshold = thresh;
  numberOfSensor = (EndPin - StartPin) + 1;
  if (Type == "digital") {
    for (int i = startPin; i <= endPin; i++) {
      pinMode(startPin, OUTPUT);
    }
  }
}

Sonar::Sonar(uint8_t trig, uint8_t echo) {
  TrigPin = trig;
  EchoPin = echo;

  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  digitalWrite(TrigPin, LOW);
}



// LED method definitions
void Led::on() {                                                              //led set to on
  digitalWrite(pinNumber, HIGH);
}
void Led::off() {                                                             //led set to off
  digitalWrite(pinNumber, LOW);
}
void Led::Blink(int onPulse, int offPulse) {                                  // Led blink
  digitalWrite(pinNumber, HIGH);
  delay(onPulse);
  digitalWrite(pinNumber, LOW);
  delay(offPulse);
}

//MOTOR method definition
void Motor::forward(uint8_t ms) {                                                  // motor forward
  analogWrite(pwmPIN, ms);
  digitalWrite(forwardPIN, HIGH);
  digitalWrite(backwardPIN, LOW);
}
void Motor::backward(uint8_t ms) {                                                 //motor backward
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
void Robot::forward(uint8_t ms) {                                                    // Lfr forward move at "ms" speed
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, HIGH);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, HIGH);
  digitalWrite(lmb, LOW);
}
void Robot::backward(uint8_t ms) {                                                  // Lfr backward move at "ms" speed
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
void Robot::Break(uint8_t ms, int Time) {                                            // Breaking system for Lfr, operation runs for "Time" miliseconds
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, HIGH);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, HIGH);
  delay(Time);
}
void Robot::rightTurn(uint8_t ms) {                                                // Lfr right turns
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, LOW);
  digitalWrite(rmb, HIGH);
  digitalWrite(lmf, HIGH);
  digitalWrite(lmb, LOW);
}
void Robot::leftTurn(uint8_t ms) {                                                       // Lfr left turns
  analogWrite(rm, ms);
  analogWrite(lm, ms);
  digitalWrite(rmf, HIGH);
  digitalWrite(rmb, LOW);
  digitalWrite(lmf, LOW);
  digitalWrite(lmb, HIGH);
}

void IrSensor::rawAnalogValue(uint16_t * raw_analog_value) {                                                           //analog value without calibration
  for (uint8_t i = startPin; i <= endPin; i++) {
    raw_analog_value[i] = analogRead(i);
  }
}
void IrSensor::analogValue(uint16_t * analog_value, uint16_t low_value = 0, uint16_t high_value = 1024) {                   // analog value after calibration
  rawAnalogValue(temp);
  for (uint8_t i = 0; i < numberOfSensor; i++) {
    analog_value[i] = map(temp[i], min_value[i], max_value[i], low_value, high_value);
  }
}
void IrSensor::digitalValue(uint16_t * digital_value, uint16_t lowValue = 0, uint16_t highValue = 1024) {
  analogValue(temp, lowValue, highValue);

  for (uint16_t i = 0; i < numberOfSensor; i++) {
    if (temp[i] < lowValue + (threshold * min_value[i])) digital_value[i] = 1;
    if (temp[i] > lowValue + (threshold * min_value[i])) digital_value[i] = 0;
  }

}
void IrSensor::squaredValue(uint16_t * squared_value, uint16_t lowValue = 0, uint16_t highValue = 1024) {
  analogValue(temp, lowValue, highValue);

  uint8_t tempo = 0;
  for (uint16_t i = 0; i < numberOfSensor; i++) {
    tempo = abs(i - (4.5)) + 1;
    if (temp[i] < lowValue + (threshold * min_value[i])) squared_value[i] = tempo * tempo;
    else squared_value[i] = 0;
  }
}
void IrSensor::Calibrate(uint16_t numberOfSample) {                                                     // sensor Calibration
  Serial.println("Calibration starts >>>");
  uint16_t temp[numberOfSensor];
  for (uint8_t i = 0; i < numberOfSensor; i++) {
    max_value[i] = 0;
    min_value[i] = 0xffff;
  }
  for (uint8_t i = 0; i < numberOfSample; i++) {
    rawAnalogValue(temp);
    for (uint8_t j = 0; j < numberOfSensor; j++) {
      max_value[j] = max(max_value[j], temp[j]);
      min_value[j] = min(min_value[j], temp[j]);
    }
    delay(30);
  }

  Serial.print("Max Value : ");                                                          //printing min, max value
  for (uint8_t i = 0; i < numberOfSensor; i++) {
    Serial.print(max_value[i]);
    Serial.print("  ");
  }
  Serial.println();
  Serial.print("Min Value : ");
  for (uint8_t i = 0; i < numberOfSensor; i++) {
    Serial.print(min_value[i]);
    Serial.print("  ");
  }
  Serial.println();
  Serial.println("Calibration done.");
}
void IrSensor::minValue(uint16_t * minV) {
  for (uint8_t i = 0; i < numberOfSensor; i++) {
    minV[i] = min_value[i];
  }
}
void IrSensor::maxValue(uint16_t * maxV) {
  for (uint8_t i = 0; i < numberOfSensor; i++) {
    maxV[i] = max_value[i];
  }
}
void IrSensor::sum(uint8_t  *line_sum, uint8_t *left_sum, uint8_t *right_sum) {
  squaredValue(temp);

  uint8_t i = 0;
  for (i; i < numberOfSensor / 2; i++) {
    line_sum += temp[i];
    left_sum += temp[i];
  }
  for (i; i < numberOfSensor; i++) {
    line_sum += temp[i];
    right_sum += temp[i];
  }
}
uint16_t IrSensor::refreshRate(uint16_t Time) {
  auto finalTime = millis() + Time;
  uint16_t counter = 0;
  while (millis() < finalTime) {
    readLine();
    ++counter;
  }
  return counter;
}
uint16_t IrSensor::sum(uint8_t s, uint8_t e, uint16_t *arr) {
  int temp = 0;
  for (int i = s; i <= e; i++) {
    temp += arr[i];
  }
  return temp;
}
int IrSensor::readLine(uint8_t lowValue = 0, uint16_t highValue = 100) {
  rawAnalogValue(temp);
  uint8_t tempo = 0;
  for (uint16_t i = 0; i < numberOfSensor; i++) {
    tempo = abs(i - (4.5)) + 1;
    temp[i] = map(temp[i], min_value[i], max_value[i], lowValue, highValue * (tempo + 1));
//    Serial.print(temp[i]);
//    Serial.print("  ");
  }
//  Serial.println();
  int Position = sum(0, 4, temp) - sum(5, 9, temp);
  return Position;
}
float Sonar::distance() {
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(10);
  auto dst = pulseIn(EchoPin, HIGH) * 0.0163;
  return dst;
}

//Special function definition
void printArray(uint16_t * arr, uint8_t numberOfElement) {
  for (uint8_t i = 0; i < numberOfElement; i++) {
    Serial.print(arr[i]);
    Serial.print("\t");
  }
  Serial.println();
}
