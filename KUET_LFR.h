#ifndef lfr
#define lfr

#if (ARDUINO >= 100)
#include<Arduino.h>
#else
#include<WProgram.h>
#endif



//Classes
class Led {                                                                     //led class
  public:
    //constructor
    Led(int pin);

    //methods
    void On();
    void Off();
    void Blink(int onPulse, int offPulse);

  private:
    int pinNumber;
};

class Motor {                                                                 //motor class
  public:
    //constructor
    Motor(int pwm, int forward, int backward);

    //methods
    void Forward(int ms);
    void Backward(int ms);
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

class Robot {
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
    void Forward(int ms);
    void Backward(int ms);
    void Break(int ms, int Time);
    void rightTurn(int ms);
    void leftTurn(int ms);
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

class irSensor{
  public:
  irSensor(String type, uint8_t StartPin, uint8_t EndPin);
  void analogValue(uint16_t * av);
  void digitalValue(uint16_t * dv);
  void squaredValue(uint16_t * sv);
  void Calibrate();
  private:  
  String Type;
  uint8_t startPin;
  uint8_t endPin;
  
};






#endif
