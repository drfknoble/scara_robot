#include <Arduino.h>
#include <Wire.h>

TwoWire encoder1 = TwoWire(0);
TwoWire encoder2 = TwoWire(1);

// This function setups the microcontroller's pins.
// Args: None
// Returns: None
void setup()
{
    encoder1.begin(GPIO_NUM_23, GPIO_NUM_22);
    encoder2.begin(GPIO_NUM_19, GPIO_NUM_18);
    
    Serial.begin(115200);

    // Configure LEDs
    pinMode(GPIO_NUM_16, OUTPUT);
    digitalWrite(GPIO_NUM_16, HIGH);
    pinMode(GPIO_NUM_17, OUTPUT);
    digitalWrite(GPIO_NUM_17, HIGH);

    // Configure stepper 1
    // Direction pin 1 (dir1)
    pinMode(GPIO_NUM_27, OUTPUT);
    digitalWrite(GPIO_NUM_27, LOW);
    // Step pin 1 (step1)
    pinMode(GPIO_NUM_14, OUTPUT);
    digitalWrite(GPIO_NUM_14, LOW);

    // Configure stepper 2
    // Direction pin 2 (dir2)
    pinMode(GPIO_NUM_25, OUTPUT);
    digitalWrite(GPIO_NUM_25, LOW);
    // Step pin 2 (step2)
    pinMode(GPIO_NUM_26, OUTPUT);
    digitalWrite(GPIO_NUM_14, LOW);

    // configure analog inputs
    pinMode(GPIO_NUM_32, INPUT);
    pinMode(GPIO_NUM_33, INPUT);

}

// This function executes the program's instructions continuously.
// Args: None
// Returns: None
void loop()
{
    // Get data from analog inputs
    int analogInput1 = analogRead(GPIO_NUM_33);
    int analogInput2 = analogRead(GPIO_NUM_32);

    // Map analog value to joint limits
    float desiredAngle1 = (200 + 2700 * (float)analogInput1 / 4096);
    float desiredAngle2 = (1600 + 2200 * (float)analogInput2 / 4096);

    // Get angle from encoder 1
    encoder1.beginTransmission(0x36);
    encoder1.write(0x0E);
    encoder1.endTransmission();

    encoder1.requestFrom(0x36, 2);
    int data1 = encoder1.read();
    data1 <<= 8;
    data1 += encoder1.read();
    float angle1 = (float)data1;

    // Get angle from encoder 2
    encoder2.beginTransmission(0x36);
    encoder2.write(0x0E);
    encoder2.endTransmission();

    encoder2.requestFrom(0x36, 2);
    int data2 = encoder2.read();
    data2 <<= 8;
    data2 += encoder2.read();
    float angle2 = (float)data2;

    // Compute difference between set point and measured angle
    float error1 = desiredAngle1 - angle1;
    float error2 = desiredAngle2 - angle2;

    // A simple PID algorithm for joints 1 and 2
    if (error1 > +1.8 / 0.09) 
    {
        // angle is less than desired angle, move clockwise
        digitalWrite(GPIO_NUM_25, HIGH);
        digitalWrite(GPIO_NUM_26, HIGH);
        delay(10);
        digitalWrite(GPIO_NUM_26, LOW);
        delay(10);
        // angle1 = angle1 + 1.8;

    }
    else if (error1 < -1.8 / 0.09)
    {
        // angle is greater than desired angle, move anticlockwise
        digitalWrite(GPIO_NUM_25, LOW);
        digitalWrite(GPIO_NUM_26, HIGH);
        delay(10);
        digitalWrite(GPIO_NUM_26, LOW);
        delay(10);
        // angle1 = angle1 - 1.8;
    }

    if (error2 > +1.8 / 0.09) 
    {
        // angle is less than desired angle, move clockwise
        digitalWrite(GPIO_NUM_27, HIGH);
        digitalWrite(GPIO_NUM_14, HIGH);
        delay(10);
        digitalWrite(GPIO_NUM_14, LOW);
        delay(10);
        // Serial.println(angle1);
        // angle2 = angle2 + 1.8;

    }
    else if (error2 < -1.8 / 0.09)
    {
        // angle is greater than desired angle, move anticlockwise
        digitalWrite(GPIO_NUM_27, LOW);
        digitalWrite(GPIO_NUM_14, HIGH);
        delay(10);
        digitalWrite(GPIO_NUM_14, LOW);
        delay(10);
        // Serial.println(angle1);
        // angle2 = angle2 - 1.8;
    }


    char buf[100];
    sprintf(buf, "%f:%f:%f, %f:%f:%f", desiredAngle1, angle1, error1, desiredAngle2, angle2, error2);
    Serial.println(buf);

}
