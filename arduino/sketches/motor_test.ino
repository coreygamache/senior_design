#include <EncoderMotor.h>

//define pin constants
const int motor1inputA = 4;
const int motor1inputB = 6;
const int motor1pwm = 5;
const int motor1sleep = 12;
const float motor1gearRatio = 21.3;
const int motor1maxRPM = 201;
const int encoder1chanA = 2;
const int encoder1chanB = 7;
const float encoder1countsPerRev = 24.0;

ComponentMotor cMotor(motor1inputA, motor1inputB, motor1pwm, motor1sleep, motor1gearRatio, motor1maxRPM);
DriveMotor dMotor(9, 10, 11, motor1gearRatio, motor1maxRPM);
Encoder enc(encoder1chanA, encoder1chanB, encoder1countsPerRev);
//EncoderMotor cEncMotor(1, 2, 3, 4, 1.0, 5, 1, 2, 1.0); //component motor version
EncoderMotor cEncMotor2(cMotor, enc); //component motor object version
//EncoderMotor dEncMotor(1, 2, 3, 1.0, 4, 1, 2, 1.0); //drive motor version
EncoderMotor dEncMotor2(dMotor, enc); //drive motor object version

long count = 0;

void setup() {
  
  //setup serial output
  Serial.flush();
  Serial.begin(9600);
  Serial.println("starting program");
  Serial.println("");
  
  //attach interrupt function to handle encoder input events
  attachInterrupt(digitalPinToInterrupt(encoder1chanA), motor1inputAevent, CHANGE);

  //cEncMotor2.stop();
  cEncMotor2.forward();
  
}

void loop() {

  delay(3000);

  //count += 300;
  //cEncMotor2.setChannelACount(count);
  //dEncMotor2.setChannelACount(count);
  
  /* COMPONENT MOTOR TESTING */
  Serial.print("motor rpm: ");
  Serial.println(cEncMotor2.getMotorRPM());

  Serial.print("motor rad/sec: ");
  Serial.println(cEncMotor2.getMotorRadPerSec());
  
  Serial.print("output rpm: ");
  Serial.println(cEncMotor2.getOutputRPM());

  Serial.print("output rad/sec: ");
  Serial.println(cEncMotor2.getOutputRadPerSec());
  

  /* DRIVE MOTOR TESTING */
  /*
  Serial.print("motor rpm: ");
  Serial.println(dEncMotor2.getMotorRPM());

  Serial.print("motor rad/sec: ");
  Serial.println(dEncMotor2.getMotorRadPerSec());
  
  Serial.print("output rpm: ");
  Serial.println(dEncMotor2.getOutputRPM());

  Serial.print("output rad/sec: ");
  Serial.println(dEncMotor2.getOutputRadPerSec());

  dEncMotor2.resetCounts();
  */
  
  Serial.println("");
  
  
}

void motor1inputAevent() {
  cEncMotor2.addChannelACount();
}

