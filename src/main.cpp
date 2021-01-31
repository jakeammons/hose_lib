#include <Arduino.h>
#include <wire.h>
#include "config.h"
#include "capstan.h"

void process_command();
void setup_communication();

// capstan constructor sets starting encoder position as zero
Capstan capstan_1(T1_DIR, T1_PWM, T1_FLT, T1_CS, MUX_ADDR, T1_ENC, KP, KI, KD, CIRCUMFERENCE, UPDATE_TIME, MAX_VELOCITY);
Capstan capstan_2(T2_DIR, T2_PWM, T2_FLT, T2_CS, MUX_ADDR, T2_ENC, KP, KI, KD, CIRCUMFERENCE, UPDATE_TIME, MAX_VELOCITY);
Capstan capstan_3(T3_DIR, T3_PWM, T3_FLT, T3_CS, MUX_ADDR, T3_ENC, KP, KI, KD, CIRCUMFERENCE, UPDATE_TIME, MAX_VELOCITY);

void setup() {
    setup_communication();
    capstan_1.init();
    capstan_2.init();
    capstan_3.init();
}

void loop() {
    process_command();
    capstan_1.update();
    capstan_2.update();
    capstan_3.update();
    Serial.print(capstan_1.get_length());
    Serial.print(" ");
    Serial.print(capstan_2.get_length());
    Serial.print(" ");
    Serial.println(capstan_3.get_length());
}

void setup_communication() {
    // change pin 3 pwm frequency to 31kHz
    TCCR3B = (TCCR3B & B11111000) | B00000001;
    // change pin 7 pwm frequency to 31kHz
    TCCR4B = (TCCR4B & B11111000) | B00000001;
    // change pin 11 pwm frequency to 31kHz
    TCCR1B = (TCCR1B & B11111000) | B00000001;
    // initialize i2c and UART
    Wire.begin();
    Serial.begin(9600);
}

void process_command()
{
    if (Serial.available() > 0) {
        String string = Serial.readString();
        // find delimiters
        uint8_t d1 = string.indexOf(",");
        uint8_t d2 = string.indexOf(",", d1+1);
        uint8_t d3 = string.indexOf(",", d2+1);
        // separate string into substrings using delimiters
        char char_buffers[4][32];
        string.substring(0,d1).toCharArray(char_buffers[0], 32);
        string.substring(d1+1,d2).toCharArray(char_buffers[1], 32);
        string.substring(d2+1,d3).toCharArray(char_buffers[2], 32);
        string.substring(d3+1).toCharArray(char_buffers[3], 32);
        uint32_t duration = atoi(char_buffers[3]);
        capstan_1.set_length(atof(char_buffers[0]), duration);
        capstan_2.set_length(atof(char_buffers[1]), duration);
        capstan_3.set_length(atof(char_buffers[2]), duration);
    }
}


