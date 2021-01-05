  void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  Serial.begin(115200);
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(12, OUTPUT);
  
  
}

char ch;
unsigned int dt = 306;
void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available() < 0)
  {
    ; // do nothing
  }
    ch = Serial.read();

    if (ch == 'J')
    {
      //delay(306);
      delay(dt);
      Serial.print("j");
      digitalWrite(12, HIGH);
                        
    }
    
    if (ch == 'K')
    {
      //delay(306);
      delay(dt);
      Serial.print("k");
      digitalWrite(12, LOW);
      
    
    }
    
   //if (ch == 'D')
   // { Serial.print("j");
   //   digitalWrite(12, HIGH);
   // }
}
