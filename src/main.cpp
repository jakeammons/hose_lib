#include <Arduino.h>
#include <wire.h>
#include "config.h"
#include "capstan.h"

void process_command();

// capstan constructor sets current encoder position as zero
Capstan capstan_1(T1_DIR, T1_PWM, T1_FLT, T1_CS, MUX_ADDR, T1_ENC, KP, KI, KD);
Capstan capstan_2(T2_DIR, T2_PWM, T2_FLT, T2_CS, MUX_ADDR, T2_ENC, KP, KI, KD);
Capstan capstan_3(T3_DIR, T3_PWM, T3_FLT, T3_CS, MUX_ADDR, T3_ENC, KP, KI, KD);

void setup()
{
    // change pin 3 pwm frequency to 31kHz
    TCCR3B = TCCR3B & B11111000 | B00000001;
    // change pin 7 pwm frequency to 31kHz
    TCCR4B = TCCR4B & B11111000 | B00000001;
    // change pin 11 pwm frequency to 31kHz
    TCCR1B = TCCR1B & B11111000 | B00000001;
    // initialize i2c and UART
    Wire.begin();
    Serial.begin(9600);
    capstan_1.init();
    capstan_2.init();
    capstan_3.init();
    capstan_1.set_angle(720);
    capstan_2.set_angle(720);
    capstan_3.set_angle(720);
}

void loop()
{
    capstan_1.update();
    capstan_2.update();
    capstan_3.update();
}

//TODO: add logic to account for negative angle input
// current behavior should treat strings with negative signs as 0
// https://www.arduino.cc/en/Reference/StringToDouble
void process_command()
{
    if (Serial.available() > 0) {
        int d1, d2;
        String string;
        string = Serial.readString();
        // find delimiters
        d1 = string.indexOf(",");
        d2 = string.lastIndexOf(",");
        // separate string into substrings using delimiters
        capstan_1.set_angle(string.substring(0,d1).toDouble());
        capstan_2.set_angle(string.substring(d1+1,d2).toDouble());
        capstan_3.set_angle(string.substring(d2+1).toDouble());
    }
}
