//includes
#include <EncoderMotor.h>
#include <PID_v1.h>
#include <Wire.h>

//define pin constants
const int motor1dir = 5;
const int motor1pwm = 6;
const int motor1sleep = 7
const int motor2dir = 9;
const int motor2pwm = 10;
const int motor2sleep = 8;
const int encoder1chanA = 2;
const int encoder1chanB = 11;
const int encoder2chanA = 3;
const int encoder2chanB = 12;

//motor parameter constants
const float motor1gearRatio = 10.0;
const float motor2gearRatio = 10.0;
const int encoder1countsPerRev = 10;
const int encoder2countsPerRev = 10;
const int motor1maxRPM = 10;
const int motor2maxRPM = 10;

//PID setting constants
const int pidKP = 0.5; //proportional value; default 2; wiki: 0.5
const int pidKI = 5; //integral value; default 5
const int pidKD = 0.1; //derivative value; default 1
const int pidSampleTime = 200; //ms

//other constants
const int msPerCycle = 200; //ms

//EncoderMotor objects int (fPin, int rPin, int maxRPM, int channelA, int channelB, int countableEventsPerRev)
EncoderMotor motor1(motor1dir, motor1pwm, motor1sleep, motor1gearRatio, motor1maxRPM, encoder1chanA, encoder1chanB, encoder1countsPerRev);
EncoderMotor motor2(motor2dir, motor2pwm, motor2sleep, motor2gearRatio, motor2maxRPM, encoder2chanA, encoder2chanB, encoder2countsPerRev);

//PID variables
double setPoint_motor1, input_motor1, output_motor1;
double setPoint_motor2, input_motor2, output_motor2;

//specify links and initial tuning parameters
PID PID_Motor1(&input_motor1, &output_motor1, &setPoint_motor1, pidKP, pidKI, pidKD, DIRECT);
PID PID_Motor2(&input_motor2, &output_motor2, &setPoint_motor2, pidKP, pidKI, pidKD, DIRECT);

void setup()
{

  //enable PID and adjust settings
  PID_Motor1.SetMode(AUTOMATIC);
  PID_Motor2.SetMode(AUTOMATIC);
  PID_Motor1.SetSampleTime(pidSampleTime);
  PID_Motor2.SetSampleTime(pidSampleTime);
  setPoint_motor1 = 0;
  setPoint_motor2 = 0;

  //initialize i2c communication with slave address 0x04
  Wire.begin(0x04);
  Wire.onReceive(receiveData);

  //attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoder1chanA), encoder1Event, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2chanA), encoder2Event, RISING);

  //initialize serial communication and notify of initialization complete
  Serial.begin(9600);
  Serial.println("Program initialization complete, beginning control routine.");
  Serial.println("");

}

void loop()
{

//setPoint = map(pidSetPoint, 0, maxRPM, 0, 255);

  //compute current iteration PID control values
  input_motor1 = motor1.getRPM();
  input_motor2 = motor2.getRPM();
  setPoint_motor1 = 10;
  setPoint_motor2 = 10;
  PID_Motor1.Compute();
  PID_Motor2.Compute();

  //output new PID control output values to motors
  motor1.forward(map(int(output_motor1), 0, motor1.getMotor().getMaxRPM(), 0, 255));
  motor2.forward(map(int(output_motor2), 0, motor2.getMotor().getMaxRPM(), 0, 255));

  //output current iteration PID input values to serial
  Serial.print("Motor 1 Input: ");
  Serial.println(input_motor1);
  Serial.print("Motor 2 Input: ");
  Serial.println(input_motor2);

  //output current iteration PID output values to serial
  Serial.print("Motor 1 Output: ");
  Serial.println(output_motor1);
  Serial.print("Motor 2 Output: ");
  Serial.println(output_motor2);
  Serial.println();

  //delay for predefined time before next cycle
  delay(msPerCycle);

}

void encoder1Event() {
  motor1.addChannelACount();
}

void encoder2Event() {
  motor2.addChannelACount();
}

//receive and set requested motor RPM setpoints
void receiveData(int byteCount) {

  //check if data is available and message is of correct size
  if (Wire.available() && byteCount == 2)
  {

    //initialize variables
    int data;
    int maxRPM;
    int setPoint;

    for (int i = 0; i < 2; i++)
    {

      //read data from i2c buffer
      data = Wire.read();

      //get maximum RPM value of respective motor
      if (i == 0)
        maxRPM = motor1.getMotor().getMaxRPM();
      else
        maxRPM = motor2.getMotor().getMaxRPM();

      if (data >= 0 && data <= maxRPM)
        setPoint = data;
      else if (data < 0)
        setPoint = 0;
      else if (data > maxRPM)
        setPoint = maxRPM;

      if (i == 0)
        setPoint_motor1 = double(setPoint);
      else
        setPoint_motor1 = double(setPoint);

    }

  }

}
