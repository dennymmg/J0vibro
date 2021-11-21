// Date: 6th November 2021

// Protocol for vibration detection
// Speaker always on
// Piezo always on with amplitude V (such that m_bias is between 0 and 2.303), say V = 4.0

// step1: Program sends a char 'I' (in phase) to the Arduino
// step2: Arduino receives 'I' and sets pins 8,9 to HIGH; resets pins 10,11 to LOW; triggers camera
// step3: N frames are taken -- bscans computed and averaged to X1
// step4: High on Pin 10 causes the phase shift of 180
// step5: the program discards the next frame from the camera
// step6: 5 frames are taken -- bscans computed and averaged to X2
// step7: displays X2 - X1
// step8: go back to step1

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    while (!Serial) 
    {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    pinMode(8, OUTPUT);   
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);   
    pinMode(11, OUTPUT);
    pinMode(7, OUTPUT);   // for camera trigger
    pinMode(13, OUTPUT); // to LED ch1=='I' --> HIGH, ch=='O' --> LOW 
        
}// end of setup function

// Global variables
char ch1;
float t1, t2;

void loop() {
  // put your main code here, to run repeatedly:
  t1 = millis();
  while(Serial.available() <= 0)
  {
    // time out of 10 seconds
    t2 = millis();
    if(t2-t1 >30) // 10 seconds
    {
      // PORT B = B0 - pin8, B1 - pin9, B2 - pin 10, ..., B4 - pin12
      if (ch1=='I') 
      {
        PORTB = B00000011; // pins 8 and 9 reset to 0, pins 10 and 11 set to 1 
        digitalWrite(13, HIGH);
      }
      else 
      {
        PORTB = B00001100; // pins 8 and 9 set to 1, pins 10 and 11 reset to 0
        digitalWrite(13, LOW);
      }
      // level 
      PORTD = PORTD | B10000000; // set the pin7 D7  (camera trigger)
      delayMicroseconds(200);
      PORTD = PORTD & B01111111;
    }   
  }
    
  ch1 = Serial.read();

  if (ch1 == 'I') // I - in phase with speaker (non-inverting op amp)
  {
    // PORT B = B0 - pin8, B1 - pin9, B2 - pin 10, ..., B4 - pin12
    PORTB = B00000011; // pins 8 and 9 set to 1, pins 10 and 11 reset to 0
    PORTD = PORTD | B10000000; // set the pin7 D7  (camera trigger)
    delayMicroseconds(200);
    PORTD = PORTD & B01111111;
    digitalWrite(13, HIGH);
  }
  
  if (ch1 == 'O')  // I - out of phase with speaker (inverting op amp)
  {
    // PORT B = B0 - pin8, B1 - pin9, B2 - pin 10, ..., B4 - pin12
    PORTB = B00001100; // pins 8 and 9 reset to 0, pins 10 and 11 set to 1 
    PORTD = PORTD | B10000000; // set the pin7 D7  (camera trigger)
    delayMicroseconds(200);
    PORTD = PORTD & B01111111;
    digitalWrite(13, LOW);
  }
                
}
