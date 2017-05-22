// generic pwm-controlled actuator class

#ifndef PWM_ACTUATOR_H
#define PWM_ACTUATOR_H

#include "Arduino.h"

class pwm_actuator
{
public:
    pwm_actuator();
    //init -- NOTE: brake pin can be used as disable pin (if enable pin is available, use brake_negate)
    void init(int pin_speed, int pin_dir, int pin_brake, int extend_speed_=255, int retract_speed_=255, int brake_negate_=0);
    void extend(int speed=-1);
    void retract(int speed=-1);
    inline void forward(int speed=-1){extend(speed);}
    inline void backwards(int speed=-1){retract(speed);}
    void stop();
private:
    int PIN_SPEED;
    int PIN_DIR;
    int PIN_BRAKE;
    int extend_speed;
    int retract_speed;
    int brake_negate;
};

//takes a speed from -1000 to 1000 and sets the motors
String roboteq_string(String speed_r, String speed_l);

#endif
