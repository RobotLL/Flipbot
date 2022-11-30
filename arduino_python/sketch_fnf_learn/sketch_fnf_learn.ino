String str;
int cmd;
int sigL = 8;      //back finger
int sigR = 9;      //Blue
int i=1;

void setup()
{ 
  pinMode(sigL, OUTPUT); 
  pinMode(sigR, OUTPUT);
  pinMode(13,OUTPUT);
  Serial.begin(9600);
}

void loop()
{       
   if (Serial.available()){
    // read from PC 
    str = Serial.readStringUntil('\n');
    cmd = str.toInt();
    digitalWrite(13, HIGH-digitalRead(13)); 
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
      delay(1000);
      analogWrite(sigR, 254);
      Serial.println("Soft Gripper Flex-and-Flip.");
    }
    else{
      analogWrite(sigL, 100); 
      delay(1000);
      analogWrite(sigR, cmd);
      Serial.println("Soft Gripper Testing.");
    }
  }
 
}
