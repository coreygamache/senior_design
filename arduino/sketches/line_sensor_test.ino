#include <QTRSensors.h>

//QTR line sensor pins and constants
const int qtrCalibrationTime = 200;
const unsigned char qtrEmitterPin = 255;
const unsigned char qtrNumSamplesPerSensor = 4;
const unsigned char qtrNumSensors = 3;

//QTR line sensor object and variables
const int qtrCenterPosition = ((qtrNumSensors - 1) * 1000) / 2;
unsigned int qtrSensorValues[qtrNumSensors] = {0};
QTRDimmableAnalog qtrSensors((unsigned char[]) {0, 1, 2}, qtrNumSensors, qtrNumSamplesPerSensor, qtrEmitterPin);


void setup() {

  Serial.flush();
  Serial.begin(9600);
  Serial.println("initializing program and starting calibration");

  for (int i = 0; i < qtrCalibrationTime; i++) {
    //-->incrementally rotate robot by some amount
    qtrSensors.calibrate();
  }

  Serial.println("calibration complete");

}

void loop() {
  Serial.print("sensor raw values: ");
  qtrSensors.read(qtrSensorValues, QTR_EMITTERS_ON);
  for (int i = 0; i < qtrNumSensors; i++) {
    Serial.print(qtrSensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  delay(1000);

  Serial.print("sensor calibrated values: ");
  qtrSensors.readCalibrated(qtrSensorValues, QTR_EMITTERS_ON);
  for (int i = 0; i < qtrNumSensors; i++) {
    Serial.print(qtrSensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  delay(2000);

}
