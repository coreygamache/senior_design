//includes
#include <Wire.h>

//define pin constants
const int LED1 = 5;
const int LED2 = 9;

//LED PWM variables
volatile double setPoint_LED1, setPoint_LED2;

//other constants
const int msPerCycle = 1; //ms

void setup()
{

  //initialize i2c communication with slave address 0x04
  Wire.begin(0x04);
  Wire.onReceive(receiveData);

  //initialize setPoint variables
  setPoint_LED1 = 0;
  setPoint_LED2 = 0;

  //initialize serial communication and notify of initialization complete
  Serial.begin(9600);
  Serial.println("Program initialization complete, beginning control routine.");
  Serial.println("");

}

void loop()
{

//setPoint = map(pidSetPoint, 0, maxRPM, 0, 255);

  //compute current iteration PID control values
  analogWrite(LED1, setPoint_LED1);
  analogWrite(LED2, setPoint_LED2);

  //delay for predefined time before next cycle
  delay(msPerCycle);

}

//receive and set requested motor RPM setpoints
void receiveData(int byteCount) {

  Serial.print("data received, byes: ");
  Serial.println(byteCount);

  //check if data is available and message is of correct size
  if (Wire.available() && byteCount == 2)
  {

    //initialize variables
    int data;
    int setPoint;

    for (int i = 0; i < byteCount; i++)
    {

      //read data from i2c buffer
      data = Wire.read();

      if (i == 0)
        Serial.print("LED 1 setPoint: ");
      else
        Serial.print("LED 2 setPoint: ");
      Serial.println(data);

      if (data >= 0 && data <= 255)
        setPoint = data;
      else if (data < 0)
        setPoint = 0;
      else if (data > 255)
        setPoint = 255;

      if (i == 0)
        setPoint_LED1 = double(setPoint);
      else
        setPoint_LED2 = double(setPoint);

    }

  }

}
