// Fault indicator led
#define errorLed A4

// Connections for motor driver cards
// Motor driver R
#define enR 2
#define inaR 3
#define inbR 4
#define pwmR 5
// Motor driver L
#define enL 6
#define inaL 7
#define inbL 8
#define pwmL 9

// Cutter motor
#define cutRun 10
// Theese two can be on one output, the motor will always run full speed (PWM=high)
#define cutPwm 11
// Use rpm out?

// Current sensing
#define mainCurrent A0 // Current sensor for electronics & cutter motor
#define motorRcurrent A2  // Motor current R
#define motorLcurrent A3  // Motor current L
//double mainAmps;

// Battery monitor
#define battVolt A1  // Battery voltage

// Connections for HC-SR04, distance detector
#define trigPin 12
#define echoPin 13
long dist;

//Measuring Current Using ACS712
int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
int RawValue= 0;
int ACSoffset = 2500; // Gets values slightly off (-0.10)

// Return values from current sensing function
static double currentValues[3];

const long interval3000 = 3000;           // interval at which to check things (milliseconds)
const long interval500 = 500;
unsigned long previousMillis3000 = 0;
unsigned long previousMillis500 = 0;

// Battery monitor
float vPow = 5.02; // Voltage at the Arduinos Vcc and Vref. 
float r1 = 1000000;  // "Top" resistor, 1Mohm.
float r2 = 470000;   // "Bottom" resistor (to ground), 470 kohm.

// Status
String status="";

void setup() {
  pinMode(enR,OUTPUT);
  pinMode(inaR,OUTPUT);
  pinMode(inbR,OUTPUT);
  pinMode(pwmR,OUTPUT);
  pinMode(enL,OUTPUT);
  pinMode(inaL,OUTPUT);
  pinMode(inbL,OUTPUT);
  pinMode(pwmL,OUTPUT);
  pinMode(cutRun,OUTPUT);
  pinMode(cutPwm,OUTPUT);
  pinMode(mainCurrent,INPUT);
  pinMode(motorRcurrent,INPUT);
  pinMode(motorLcurrent,INPUT);
  pinMode(battVolt,INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode(errorLed,OUTPUT);

  Serial.begin (57600);
  Serial.println("Welcome to Lawnmower16!"); 

  // Test errorLed
  digitalWrite(errorLed, HIGH);
  delay(500);
  digitalWrite(errorLed, LOW);
}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
      int command = Serial.parseInt();
      Serial.println("Got serial data");
      // look for the newline. That's the end of your sentence:
      if (Serial.read() == '\r') {
        Serial.print("Got: ");
        Serial.println(command);
        if (command==1) {
          Serial.print ("Run!");
          status="forward";
        }
        else if (command==2) {
          Serial.print ("Stop!");     
          status="stop";     
        }
      }
  }

  // Check status
  if (status == "forward") {
    forward();
  }
  else if (status == "stop") {
    digitalWrite(inaR, LOW); // Stop
    digitalWrite(inbR, LOW);
    digitalWrite(inaL, LOW);
    digitalWrite(inbL, LOW);
  }

  
  // Timer to do checks with a fixed interval
  unsigned long currentMillis = millis();
  // Every third second
  if (currentMillis - previousMillis3000 >= interval3000) {
    // save the last time 
    previousMillis3000 = currentMillis;
    float battv = checkBatt();

    Serial.println("1000mS");
    Serial.print("Battery: ");
    Serial.print(battv/10);
    Serial.println("V");
    
    Serial.print("Distance: ");
    Serial.println(dist);

    Serial.print("Main amps: ");
    Serial.println(currentValues[0]);
    Serial.print("Motor R amps: ");
    Serial.println(currentValues[1]);
    Serial.print("Motor L amps: ");
    Serial.println(currentValues[2]);

  }
  if (currentMillis - previousMillis500 >= interval500) {
    // save the last time 
    previousMillis500 = currentMillis;
    //Serial.println("500mS");
    
    // Distance
    dist = distance();
    // Check current
    checkCurrent();
    
  }
}

void forward() {
    analogWrite(pwmR, 100); // Set speed
    analogWrite(pwmL, 100); // Set speed
    digitalWrite(inaR,HIGH); // Rotate Cw
    digitalWrite(inbR, LOW);
    digitalWrite(enR, HIGH);
    digitalWrite(inaL, HIGH); // Rotate Cw
    digitalWrite(inbL, LOW);
    digitalWrite(enL, HIGH);
}

int checkCurrent() {
    int adcvalueCR = analogRead(motorRcurrent);  
    int adcvalueCL = analogRead(motorLcurrent);  
    int adcvalueCM = analogRead(mainCurrent);  // Cutter motor + electronics

    // Main   
    double Voltage = (adcvalueCM / 1024.0) * 4098; // Gets you mV
    double Amps = ((Voltage - ACSoffset) / mVperAmp);

    // My ACS is backwards... Adjust value
    double Amps2 = Amps + 5;
    currentValues[0] = (-Amps)-2.5;

    // Todo: Calculate real ampere value
    currentValues[1] = adcvalueCR;
    currentValues[2] = adcvalueCL;

    /*
    Serial.print("Amps ADC value: ");
    Serial.println(RawValue);
    Serial.print("Calculated amps ");
    Serial.println(Amps,2);
    Serial.println(Amps2,2);
    Serial.println(amps3,2);

    Serial.print("adcvalueCM: ");
    Serial.println(adcvalueCM);
    Serial.print("adcvalueCR: ");
    Serial.println(adcvalueCR);
    Serial.print("adcvalueCL: ");
    Serial.println(adcvalueCL);
    */
    return currentValues;
}

int checkBatt() {
  int adcvalue = analogRead(battVolt);  

  //vPow = 15;
  int volt = (adcvalue * vPow) / 1024.0;
  int volt2 = volt / (r2 / (r1 + r2));
  //Serial.print("volt: ");
  //Serial.println(volt2);
  float v = (adcvalue * vPow) / 1024.0;
  float v2 = v / (r2 / (r1 + r2));
  // Correction
  v2=v2-0.2;
  /*
  Serial.print("Battery monitor ADC: ");
  Serial.println(adcvalue);
  Serial.print("Battery voltage: ");
  Serial.print(v2);
  Serial.println(" volt.");
  */
  // Convert to int
  v2=v2*10;
  int batt=(int) v2;
  
  return batt;
}
long distance() {
  // Measure distance to sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  return distance;
}

