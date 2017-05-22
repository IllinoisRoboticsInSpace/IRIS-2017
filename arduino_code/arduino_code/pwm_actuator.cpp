// generic pwm-controlled actuator class

#include "pwm_actuator.h"

pwm_actuator::pwm_actuator():PIN_SPEED(-1){}

void pwm_actuator::init(int pin_speed, int pin_dir, int pin_brake, int extend_speed_, int retract_speed_, int brake_negate_)
{
    PIN_SPEED=pin_speed;
    PIN_DIR=pin_dir;
    PIN_BRAKE=pin_brake;
    extend_speed=extend_speed_;
    retract_speed=retract_speed_;
    brake_negate=brake_negate_;
    
    pinMode(PIN_SPEED, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_BRAKE, OUTPUT);
    
    analogWrite(PIN_SPEED, 0);
    digitalWrite(PIN_DIR, LOW);          //LOW - Retract, HIGH - Extend
    digitalWrite(PIN_BRAKE, HIGH != brake_negate);
}

void pwm_actuator::extend(int speed)
{
    if(speed==-1)speed=extend_speed;
    digitalWrite(PIN_BRAKE, LOW != brake_negate);
    digitalWrite(PIN_DIR, HIGH);        //LOW - Retract, HIGH - Extend
    analogWrite(PIN_SPEED, speed);        //255 = 100 % PWM
}

void pwm_actuator::retract(int speed)
{
    if(speed==-1)speed=retract_speed;
    digitalWrite(PIN_BRAKE, LOW != brake_negate);
    digitalWrite(PIN_DIR, LOW);     //LOW - Retract, HIGH - Extend
    analogWrite(PIN_SPEED, speed);        //255 = 100% PWM
}

void pwm_actuator::stop()
{
    analogWrite(PIN_SPEED, 0);          //Stop actuator
    digitalWrite(PIN_BRAKE, HIGH != brake_negate);
}

//takes a speed from -1000 to 1000 and sets the motors
String roboteq_string(String speed_r, String speed_l)
{
  //build string string
  String out = "!G 1 ";
    out += speed_r;
    out += "_";
    out += "!G 2 ";
  out += speed_l;
    out += "_\n";
  return out;
}
