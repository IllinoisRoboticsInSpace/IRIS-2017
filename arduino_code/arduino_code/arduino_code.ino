#include <Wire.h>
#include "pwm_actuator.h"
#include "iris_pinout.h"
#include "stepper_pulse.h"
#include <Servo.h>

//#include "FreeSixIMU.h"
 
//Call the constructors
pwm_actuator webcam_linear = pwm_actuator();
pwm_actuator bin_linear = pwm_actuator();
pwm_actuator collect_linear = pwm_actuator();

Servo webcam_servo;

int pos = 0;
//FreeSixIMU imu = FreeSixIMU();
//stepper_pulse webcam_stepper = stepper_pulse();

void update_time_critical()
{
    //Update servos
//    webcam_stepper.update_pulses(); //send remaining servo pulses
    //Update IMU/ Not implemented yet
    //imu.getQ(0); // give null pointer, just to force calculation, no need for values
}


void setup()
{
    webcam_servo.attach(SERVO_PULSE);
    //Maxon pinout
    pinMode(MAXON_BRAKE, OUTPUT); //unset brake
    webcam_linear.init(
        MAXON_SPEED, MAXON_DIR, MAXON_ENABLE, //pins
        0, // extend_speed (forward)
        192, // retract_speedint (backwards)
        1 //brake_negate (enable negate)
        );
    pinMode(MAXON_HALL, INPUT);

    //Dispensation LA pinout
    bin_linear.init(
        DISP_SPEED, DISP_DIR, DISP_BRAKE, //pins
        194, // extend_speed (forward)
        194, // retract_speedint (backwards)
        0 //brake_negate (enable negate)
        );
    pinMode(DISP_THERMAL, INPUT);
    pinMode(DISP_HALL, INPUT);
    pinMode(DISP_POT, INPUT);
    pinMode(DISP_CURRENT, INPUT);
    digitalWrite(A0, HIGH);             //Set the pullup resistor

    //Paddle LA pinout
    collect_linear.init(
        ARM_SPEED, ARM_DIR, ARM_BRAKE, //pins
        127, // extend_speed (forward)
        127, // retract_speedint (backwards)
        0 //brake_negate (enable negate)
        );
    pinMode(ARM_THERMAL, INPUT);
    pinMode(ARM_POT, INPUT);
    pinMode(ARM_CURRENT, INPUT);
    
    //Webcam stepper motor
//    webcam_stepper.init(
//        SERVO_DIR, SERVO_PULSE, //pins
//        25  // interval millisecond (between pulses)
//        );
        
    //Serial communications
    Serial.begin(9600);               //ODroid Serial
    Serial.setTimeout(0);  // never wait for anything
//    Serial1.begin(115200);              //Roboteq Serial

    //I2C communications
    Wire.begin();

    //Initialize IMU
    //imu.init();  

    Serial.println("Ready");
}

void loop()
{
    //input handling
    int commaIdx[3];
    int idx_start = 0;
    int idx_end = 0;
    int idx = 0;
    bool data = false; //Checks if Data ends and is valid.
    String str; //input from the serial
    //Robot Status
    int check[4]; //checks if message is valid
    
    //Wait for data to arrive (Format: "collect_linear_act,bin_linear_act,webcam_linear_act,servo_degrees#!")
    // 0,0,0,000#!
    while(Serial.available() > 0)
    {
        str = "";
        int c=Serial.read(); // get next char
        while(c!='!')
        {
            if(c>0) // check valid readf
                str += (char) c; // append to string
            //update_time_critical();
            c=Serial.read(); // get next char
        }
        idx_start = 0;
        idx_end = str.length();

        // Check we got good data
        data = str.endsWith("#");
    }
    
    if(data)
    {
        //reset data flag
        data = false;
        
        //Find the index of the commas
        commaIdx[0] = str.indexOf(',', idx_start);
        commaIdx[1] = str.indexOf(',', commaIdx[0]+1);
        commaIdx[2] = str.indexOf(',', commaIdx[1]+1);

                
        int disp_ind = commaIdx[0] +1;
        int collection_ind = 0;
        int web_act_ind = commaIdx[1] +1;
        int servo_ind = commaIdx[2]+1;

        
        //update_time_critical();
        
        //Handle collection LA commands
        if(str[collection_ind] == '0')
        {
            int dist = analogRead(A14);
            collect_linear.extend();

        }
        else if(str[collection_ind] == '1')
        {
            collect_linear.stop();
        }
        else if(str[collection_ind] == '2')
        {
            int dist = analogRead(A14);
            collect_linear.retract();
        }

        //Handle bin LA commands
        if(str[disp_ind] == '0')
        {
            bin_linear.retract();
        }
        else if(str[disp_ind] == '1')
        {
            bin_linear.stop();
        }
        else if(str[disp_ind] == '2')
        {
            bin_linear.extend();
        }

        //Handle webcam Actuator commands
        if(str[web_act_ind] == '0')
        {
            bin_linear.retract();
        }
        else if(str[web_act_ind] == '1')
        {
            bin_linear.stop();
        }
        else if(str[web_act_ind] == '2')
        {
            bin_linear.extend();
        }
        
        //Handle Servo
        pos = (str.substring(servo_ind, str.length())).toInt();
        webcam_servo.write(pos);


        //Update the robot status
        //Format: "disp_act, collection_act, webcam_act"
        
        check[1] = str[disp_ind]=='0'?0:(str[disp_ind]=='1'?1:2);
        check[0] = str[collection_ind]=='0'?0:(str[collection_ind]=='1'?1:2);
        check[2] = str[web_act_ind]=='0'?0:(str[web_act_ind]=='1'?1:2);
        
        check[3] = pos;
    
        //Create the string to send
        String status_str;
        for(int i=0;i<4;i++)
        {
          status_str += i?",":"";
          status_str += String(check[i]);
        }
        status_str += "#!\n";
        Serial.write(status_str.c_str(), status_str.length());
        Serial.flush();
    }
}
