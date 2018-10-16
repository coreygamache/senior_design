#include <ComponentMotor.h>
#include <DriveMotor.h>
#include <Encoder.h>
#include <EncoderMotor.h>

ComponentMotor cMotor(1, 2, 3, 4, 1.0, 5);
DriveMotor dMotor(1, 2, 3, 1.0, 4);
Encoder enc(1, 2, 3);
EncoderMotor cEncMotor(1, 2, 3, 4, 1.0, 5, 1, 2, 1.0); //component motor version
EncoderMotor cEncMotor2(cMotor, enc); //component motor object version
EncoderMotor dEncMotor(1, 2, 3, 1.0, 4, 1, 2, 1.0); //drive motor version
EncoderMotor dEncMotor2(dMotor, enc); //drive motor object version

void setup() {
    
}

void loop() {
    
}
