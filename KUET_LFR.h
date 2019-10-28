#ifndef lfr
#define lfr

#if (ARDUINO >= 100)
#include<Arduino.h>
#else
#include<WProgram.h>
#endif



//Classes
class Led {                                                                     //led 
  public:
    //constructor
    Led(int pin);

    //methods
    void on();
    void off();
    void Blink(int onPulse, int offPulse);

  private:
    int pinNumber;
};

class Motor {                                                                 //motor
  public:
    //constructor
    Motor(uint8_t pwm, uint8_t forward, uint8_t backward);

    //methods
    void forward(uint8_t ms);
    void backward(uint8_t ms);
    void Stop();
    
    int getPwmPin() const {
      return pwmPIN;
    }
    int getForwardPin() const {
      return forwardPIN;
    }

    int getBackwardPin() const {
      return backwardPIN;
    }
  private:
    int pwmPIN;
    int forwardPIN;
    int backwardPIN;

};

class Robot {                                                                     //Robot
  public:
    Robot(const Motor &m1, const Motor &m2);
//      : rm{m1.getPwmPin()},
//        lm{m2.getPwmPin()},
//        rmf{m1.getForwardPin()},
//        lmf{m2.getForwardPin()},
//        rmb{m1.getBackwardPin()},
//        lmb{m2.getBackwardPin()}
//    {
////      pinMode(rm, OUTPUT);
////      pinMode(lm, OUTPUT);
////      pinMode(rmf, OUTPUT);
////      pinMode(lmf, OUTPUT);
////      pinMode(rmb, OUTPUT);
////      pinMode(lmb, OUTPUT);
//    }

    //methods
    void forward(uint8_t ms);
    void backward(uint8_t ms);
    void Break(uint8_t ms, int Time);
    void rightTurn(uint8_t ms);
    void leftTurn(uint8_t ms);
    void Stop();


  private:
    const int rm;
    const int lm;
    const int rmf;
    const int lmf;
    const int rmb;
    const int lmb;

    //  MOTOR left,right;

};

class IrSensor{                                                               //irSensor
  
  public:
  IrSensor(String type, uint8_t StartPin, uint8_t EndPin, float thresh);
  void rawAnalogValue(uint16_t * raw_analog_value);
  void analogValue(uint16_t * digital_value, uint16_t low_value = 0, uint16_t high_value = 1024);
  void digitalValue(uint16_t * digital_value, uint16_t lowValue = 0, uint16_t highValue = 1024);
  void squaredValue(uint16_t * squared_value, uint16_t lowValue = 0, uint16_t highValue = 1024);
  void Calibrate(uint16_t number_of_sample);
  void minValue(uint16_t * min_value);
  void maxValue(uint16_t * max_value);
  void sum(uint8_t  *line_sum, uint8_t *left_sum, uint8_t *right_sum);
  uint16_t refreshRate(uint16_t Time);
  int readLine(uint8_t lowValue = 0, uint16_t highValue = 100);
  uint16_t sum(uint8_t s, uint8_t e, uint16_t *arr);
 
  
  private:  
  String Type;
  uint8_t startPin;
  uint8_t endPin;
  int temp[15];
  float threshold;
  uint8_t numberOfSensor;
  uint16_t numberOfSample;
  uint16_t min_value[15];
  uint16_t max_value[15];
  
};

class Sonar{
  public:
  Sonar(uint8_t trig, uint8_t echo);
  float distance();

  private:
  uint8_t TrigPin;
  uint8_t EchoPin;
};

//Special functions
void printArray(uint16_t * arr, uint8_t numberOfElement);

#endif
