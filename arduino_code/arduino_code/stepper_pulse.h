//asychronous pulse management
// stepper_pulse::update_pulses needs to be called **as often as possible**

#define TICKS_PER_DEGREE 0.551

class stepper_pulse
{
public:
    stepper_pulse():current(0),target(0),last_pulse_millis(0),SERVO_DIR_PIN(-1){}
    inline void init(int servo_dir_pin,int pulse_dir_pin,int interval_millis_)
    {
        SERVO_DIR_PIN = servo_dir_pin; SERVO_PULSE_PIN = pulse_dir_pin;
        interval_millis=interval_millis_;
        pinMode(SERVO_DIR_PIN,OUTPUT);
        pinMode(SERVO_PULSE_PIN,OUTPUT);
    }
    inline void setTarget(float degrees)
    {
        while(degrees>360.0)
            degrees-=360.0;
        while(degrees<0.0)
            degrees+=360.0;
        if(current==target)last_pulse_millis=millis();
        target=degrees*(TICKS_PER_DEGREE);
    }
    inline float getCurrent(){return current*1.0/(TICKS_PER_DEGREE);}
    inline float getMoving(){return current != target;}
    void update_pulses();
private:
    int current;
    int target;
    int sense;
    unsigned long last_pulse_millis;
    int interval_millis;
    int SERVO_DIR_PIN;
    int SERVO_PULSE_PIN;
};


void stepper_pulse::update_pulses()
{
    unsigned long now=millis();
    while(current != target && now-last_pulse_millis>interval_millis)
    {
        if(current<target)
        {
          digitalWrite(SERVO_DIR_PIN,HIGH);
          current++;
        }
        else if(current>target)
        {
          digitalWrite(SERVO_DIR_PIN,LOW);
          current--;
        }        
        digitalWrite(SERVO_PULSE_PIN,HIGH);
        delayMicroseconds(50);
        digitalWrite(SERVO_PULSE_PIN,LOW);         
        delayMicroseconds(50);  
        last_pulse_millis+=interval_millis;
    }
}
