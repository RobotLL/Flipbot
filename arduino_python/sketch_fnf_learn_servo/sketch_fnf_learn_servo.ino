#include <Servo.h>

String str;
int cmd;
int sigL = 8;      //back finger
int sigR = 9;      //Blue
int i=1;
int pos = 90;
Servo servo;

void setup()
{ 
  pinMode(sigL, OUTPUT); 
  pinMode(sigR, OUTPUT);
  servo.attach(12);
  Serial.begin(9600);
}

void loop()
{       
   if (Serial.available()){
    // read from PC 
    str = Serial.readStringUntil('\n');
    cmd = str.toInt();
   if(cmd == 0){
      analogWrite(sigL, 0); 
      delay(100);
      analogWrite(sigR, 0);
      Serial.println("Soft Gripper Open.");
    }
    else if(cmd == 2){      
      analogWrite(sigL, 180); 
      delay(1);
      analogWrite(sigR, 180);
      Serial.println("Soft Gripper CLosed.");
    }
    else if(cmd == 1){      
      analogWrite(sigL, 150); 
      delay(500);
      analogWrite(sigR, 254);
      Serial.println("Soft Gripper Flex-and-Flip.");
    }
    else if(cmd == 3){      
//      servo.write(0);
//      delay(500);
//      Serial.println("Servo close.");
  for (pos = 90; pos >= 0; pos -= 5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
    }
    else if(cmd == 4){      
//      servo.write(180);
//      delay(500);
//      Serial.println("Servo open.");
  for (pos = 90; pos <= 180; pos += 5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
    }
    else{
      analogWrite(sigL, 100); 
      delay(1000);
      analogWrite(sigR, cmd);
      Serial.println("Soft Gripper Testing.");
    }
  }
 
}
