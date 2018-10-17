#include <EncoderMotor.h>
#include <PID_v1.h>

//define motor pins and constants
const int encoder1chanA = 2, encoder1chanB = 7;
const int motor1inputA = 4, motor1inputB = 6, motor1pwm = 5, motor1sleep = 12;
const int motor1maxRPM = 201;
const float encoder1countsPerRev = 64.0, motor1gearRatio = 21.3;

//PID setting constants
const double pidKP = 0.5; //proportional value; default 2; wiki: 0.5
const double pidKI = 1.0; //integral value; default 5
const double pidKD = 0.02; //derivative value; default 1
const int pidSampleTime = 200; //ms

//other constants
const int msPerCycle = 50; //ms
const int displayStatusFrequency = 1000; //ms

//PID variables and objects
double setPoint_motor1, input_motor1, output_motor1;
PID PID_Motor1(&input_motor1, &output_motor1, &setPoint_motor1, pidKP, pidKI, pidKD, DIRECT);

//motor objects
ComponentMotor cMotor(motor1inputA, motor1inputB, motor1pwm, motor1sleep, motor1gearRatio, motor1maxRPM);
DriveMotor dMotor(9, 10, 11, motor1gearRatio, motor1maxRPM);
Encoder enc(encoder1chanA, encoder1chanB, encoder1countsPerRev);
EncoderMotor cEncMotor(cMotor, enc); //component motor object version
//EncoderMotor dEncMotor(dMotor, enc); //drive motor object version

//other variables
int displayStatusCount = 0;

void setup() {

  //setup serial output
  Serial.flush();
  Serial.begin(9600);
  Serial.println("starting program");
  Serial.println("");

  //attach interrupt function to handle encoder input events
  attachInterrupt(digitalPinToInterrupt(encoder1chanA), motor1inputAevent, CHANGE);

  setPoint_motor1 = 100.0;
  cEncMotor.forward(cMotorRPMtoPWM(setPoint_motor1));
  //input_motor1 = cEncMotor.getOutputRPM();

  //enable PID and adjust settings
  PID_Motor1.SetMode(AUTOMATIC);
  PID_Motor1.SetSampleTime(pidSampleTime);

}

void loop() {

  delay(msPerCycle);

  //set current iteration PID input value and calculate new output value
  input_motor1 = cEncMotor.getOutputRPM();
  bool newOutput = PID_Motor1.Compute();

  //increment display counter and display information if required
  displayStatusCount++;
  if ((displayStatusCount * msPerCycle) >= displayStatusFrequency) {

    /* DISPLAY PID CONTROLLER TESTING DATA */
    Serial.print("output rpm: ");
    Serial.println(cEncMotor.getOutputRPM());

    /*Serial.print("motor rpm: ");
    Serial.println(cEncMotor.getMotorRPM());

    Serial.print("gear ratio: ");
    Serial.println(cEncMotor.getComponentMotor().getGearRatio());

    unsigned long elapsedTime = cEncMotor.getElapsedTime();
    Serial.print("motor elapsed time: ");
    Serial.println(elapsedTime);

    elapsedTime = millis();
    Serial.print("test elapsed time: ");
    Serial.println(elapsedTime);
    */
    Serial.print("PID input (rpm): ");
    Serial.println(input_motor1);

    Serial.print("PID output (rpm): ");
    Serial.println(output_motor1);

    Serial.print("PID output (pwm): ");
    Serial.println(cMotorRPMtoPWM(output_motor1));

    //reset display status count
    displayStatusCount = 0;

    Serial.println("");
    /* END DISPLAY PID CONTROLLER TESTING DATA */

  }

  //output new PID control output values to motors
  if (newOutput) {
    cEncMotor.forward(cMotorRPMtoPWM(output_motor1));
  }

}

void motor1inputAevent() {
  cEncMotor.addChannelACount();
}

int cMotorRPMtoPWM(double RPM) {
  int pwmValue = 255 * (RPM / cEncMotor.getComponentMotor().getMaxRPM());
  if (pwmValue < 0)
    pwmValue = 0;
  else if (pwmValue > 255)
    pwmValue = 255;
  return pwmValue;
}
