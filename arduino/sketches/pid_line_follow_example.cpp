#include <pololu/orangutan.h>

unsigned char qtr_rc_pins[] = {IO_C0, IO_C1, IO_C2, IO_C3, IO_C4, IO_C5};
unsigned char qtr_rc_count  = 6;
unsigned int  qtr_rc_values[6] = {0};

int main()
{
    set_digital_input(IO_D4, PULL_UP_ENABLED);

    while(is_digital_input_high(IO_D4)) {} // Wait for user to press button

    qtr_rc_init(qtr_rc_pins,qtr_rc_count, 2000, 255);  // 800 us timeout, no emitter pin

    red_led(1);
    delay(1000);

    int i;
      for (i = 0; i < 100; i++)  // make the calibration take about 2 seconds
      {
        if (i%25==0)
        {
            if((i/5)%2==0) set_motors(20,-20);
                else set_motors(-20,20);
        }

        if (i%5)
        {
            if ((i/5)%2==0) red_led(0);
                else red_led(1);
        }
        qtr_calibrate(QTR_EMITTERS_ON);
        delay(20);
      }

    set_motors(0,0);
    red_led(1);

    while (is_digital_input_high(IO_D4)) {} // Wait for user to press button
    delay(1000);

    while (1)
    {
        //follow(); // If you want to use non PID method
        followPID();
    }

}

// Ideas is to keep line in the center
void follow()
{
    int position = qtr_read_line(qtr_rc_values, QTR_EMITTERS_ON);

    int center = (( (qtr_rc_count - 1) * 1000) / 2);
    int error  = position - center;

    int leftMotorSpeed = 50;
    int rightMotorSpeed = 50;

    if (error < -(center/2) )  // the line is on the left
    leftMotorSpeed = 0;  // turn left
    if (error > (center/2) )  // the line is on the right
    rightMotorSpeed = 0;  // turn right

    set_motors(leftMotorSpeed,rightMotorSpeed);

}


float KP = 3 ,KI = 50000 , KD = 16/1;

int   integral  = 0;
int   last_proportional = 0;

void followPID()
{
    int position = qtr_read_line(qtr_rc_values, QTR_EMITTERS_ON);
    int center = (( (qtr_rc_count - 1) * 1000) / 2);
    int proportional = position - center;
    int derivative = proportional - last_proportional;
    int power_difference = proportional / KP + integral / KI + derivative * KD;
    last_proportional = proportional;
    integral += proportional;

    const int max = 200;
    const int max_diffrence = 20;
    const int factor_diffrence = 2;

    if (power_difference > max)
        power_difference = max;
    if (power_difference < -max)
        power_difference = -max;

    // if diffrence is too much robot skids

    int leftMotorSpeed  = max;
    int rightMotorSpeed = max-power_difference;

    if(power_difference < 0)
    {
        leftMotorSpeed  = max+power_difference;
        rightMotorSpeed = max;
    }

    if (leftMotorSpeed - rightMotorSpeed > max_diffrence)
        leftMotorSpeed -= (leftMotorSpeed - rightMotorSpeed)/factor_diffrence;
    else if (rightMotorSpeed - leftMotorSpeed > max_diffrence)
        rightMotorSpeed -= (rightMotorSpeed - leftMotorSpeed)/factor_diffrence;

    set_motors(leftMotorSpeed,rightMotorSpeed);

}
