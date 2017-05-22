// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.


#include <Servo.h> 
#include <stdlib.h>
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  Serial.begin(9600);  
  //Serial.println("--- Start Serial Monitor SEND_RCVE ---");
  //Serial.println(" Type in Box above, . ");
  //Serial.println("(Decimal)(Hex)(Character)");  
  //Serial.println(); 
} 

int get_angle(){
  int angle = 0;
  int readchars = 0;
  unsigned char ByteReceived = 0;
  while(!readchars){
	  while(1){
		while(!Serial.available());
		ByteReceived = Serial.read();
                Serial.write(ByteReceived);
                Serial.write('\n');
		if(ByteReceived<'0' || ByteReceived>'9') break;
		readchars++;
		angle=angle*10+ByteReceived-'0';
	  }
  }
  Serial.print("Recieved: ");
  Serial.print(angle);
  Serial.print("\n");
  return angle;
}
 
 
void loop() 
{ 
  pos = get_angle();
  myservo.write(pos);              // tell servo to go to position in variable 'pos' 
  delay(15);                       // waits 15ms for the servo to reach the position  
} 
