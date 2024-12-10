// (c) Michael Schoeffler 2017, http://www.mschoeffler.de

#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <SoftwareSerial.h>
#include <NewPing.h>

SoftwareSerial mySerial(9, 6); 

#define tch  8 
#define IR   7

#define in1Pin 2
#define in2Pin 3
#define in3Pin 4
#define in4Pin 5

#define TRIGGER_PIN 12 // Common trigger pin for both sensors
#define ECHO_PIN_1 11   // Echo pin for the first sensor
#define ECHO_PIN_2 10   // Echo pin for the second sensor
#define MAX_DISTANCE 10000

NewPing sonar1(TRIGGER_PIN, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN, ECHO_PIN_2, MAX_DISTANCE);

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y; // variables for accelerometer raw data


  String phoneNumbers[] = {
   // "+923133375662", // Add more phone numbers here
    "+923123242801",
    "+923093913944",
    "+923073086688"
    // Add more phone numbers as needed
  };

  String message = "Battery Low, Plz Charge the Wheelchair";

char tmp_str[7]; // temporary variable used in convert function

const int batteryPin = A0;      // Analog pin for voltage monitoring
const float R1 = 100000.0;      // Resistance of the first resistor (ohms)
const float R2 = 100000.0;      // Resistance of the second resistor (ohms)
const float voltageReference = 5.0;  // Arduino's reference voltage
const float lowBatteryThreshold = 7.0;  // Adjust as needed

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}




void motorControl(int speedMotorA, int speedMotorB) {
  // Map the speed values from -255 to 255 to a range from 0 to 255
  int pwmA = map(abs(speedMotorA), 0, 255, 0, 255);
  int pwmB = map(abs(speedMotorB), 0, 255, 0, 255);

  // Motor A
  if (speedMotorA >= 0) {
    analogWrite(in1Pin, pwmA);
    digitalWrite(in2Pin, LOW);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, pwmA);
  }

  // Motor B
  if (speedMotorB >= 0) {
    analogWrite(in3Pin, pwmB);
    digitalWrite(in4Pin, LOW);
  } else {
    analogWrite(in3Pin, 0);
    analogWrite(in4Pin, pwmB);
  }
}



void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

   pinMode(IR,INPUT);
   pinMode(tch,INPUT);
  
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  
  // Turn off motors - Initial state
   motorControl(0, 0);
}
void loop() {

  
  int tchread = digitalRead(tch);
  int IRread = digitalRead(IR);
  int rawValue = analogRead(batteryPin);
  float voltage = (rawValue / 1023.0) * voltageReference * (1 + R1 / R2);
 // unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)

  if (voltage > lowBatteryThreshold){
  if(tchread == 1  && IRread == 0 ){
 if(accelerometer_y > 9000  && distance2 > 50){
  
 motorControl(150, 150);

   }

 else if(accelerometer_y < -6000){
  
 motorControl(-100, -100);
 }
 
  else if(accelerometer_x < -6000){

  motorControl(0, 150);
  }

  
  else if(accelerometer_x > 6000){
    motorControl(150, 0);
  }
  
  else{ 
  motorControl(0, 0);
} }

else{
    motorControl(0, 0);
}
  }
else{
  motorControl(0, 0);
  for (int i = 0; i < sizeof(phoneNumbers) / sizeof(phoneNumbers[0]); i++) {
    sendSMS(phoneNumbers[i], message);
//    delay(2000); // Delay between messages
  }
}
   
 
  // print out data
//  Serial.print("aX = "); Serial.println(convert_int16_to_str(accelerometer_x));
//  Serial.print(" | aY = "); Serial.println(convert_int16_to_str(accelerometer_y));
  
  // delay
//  delay(500);

 //Serial.println(distance1);
// Serial.println(distance2);
  Serial.println(voltage);
  delay(500);
// Serial.println(IRread);
//Serial.println(tchread);
}

void sendSMS(String phoneNumber, String message) {
  mySerial.print("AT+CMGS=\"");
  mySerial.print(phoneNumber);
  mySerial.println("\"");

  delay(500);
  mySerial.print(message);
  delay(100);
  mySerial.write(26); // Ctrl+Z character to send the message
  delay(500);

  updateSerial();
}


void updateSerial() {
  delay(500);
  while (Serial.available()) {
    mySerial.write(Serial.read());
  }
  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
