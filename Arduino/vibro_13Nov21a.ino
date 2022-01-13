// Date: January 2022

// Protocol for vibration detection
// Speaker always on
// Piezo always on with amplitude V (such that m_bias is between 0 and 2.405), say V = 5.0

// step1: Program sends a char 'I' (in phase) to the Arduino
// step2: Arduino receives 'I' and sets pins 8,9 to HIGH; resets pins 10,11 to LOW; triggers camera
// step3: One in-phase frame taken, bscan computed
// step4: Program sends a char 'O' (in phase) to the Arduino
// step5: Arduino receives 'O' and resets pins 8,9 to LOW; sets pins 10,11 to HIGH; triggers camera
// step6: One out-of-phase frame taken, bscan computed
// step7: Steps 1 through 6 repeated N times-- N in-phase bscans averaged to X1 and 
//                                              N out-of-phase bscans averaged to X2
// step8: compute vibration amplitude
// step9: go back to step1

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
    // time out of 3 seconds
    t2 = millis();
    if(t2-t1 > 50) // 10000 milliseconds (timeout)
    {
      // level trigger
      PORTD = PORTD | B10000000; // set the pin7 D7  (camera trigger)
      delayMicroseconds(200);
      PORTD = PORTD & B01111111;
      t1 = t2;
     }   
  }
    
  ch1 = Serial.read();

  if (ch1 == 'I') // I - in phase with speaker (non-inverting op amp)
  {
    // PORT B = B0 - pin8, B1 - pin9, B2 - pin 10, ..., B4 - pin12
    PORTB = B00000011; // pins 8,9 set to 1 (switches S3&S4 ON), pins 10,11 reset to 0 (S1&S2 OFF) 
    PORTD = PORTD | B10000000; // set the pin7 D7  (camera trigger)
    delayMicroseconds(200);
    PORTD = PORTD & B01111111;
    digitalWrite(13, HIGH);
  }
  
  if (ch1 == 'O')  // O - out of phase with speaker (inverting op amp)
  {
    // PORT B = B0 - pin8, B1 - pin9, B2 - pin 10, ..., B4 - pin12
    PORTB = B00001100; // pins 8,9 reset to 0 (switches S3&S4 OFF) , pins 10,11 set to 1 (S1&S2 ON)  
    PORTD = PORTD | B10000000; // set the pin7 D7  (camera trigger)
    delayMicroseconds(200);
    PORTD = PORTD & B01111111;
    digitalWrite(13, LOW);
  }
                
}
