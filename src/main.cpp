#include <Arduino.h>
#include <wire.h>
#include "capstan.h"
#include "config.h"
#include "hose.h"

void process_command();
void setup_communication();

// capstan constructor sets starting encoder position as zero
Capstan capstan_1(T1_DIR, T1_PWM, T1_FLT, T1_CS, MUX_ADDR, T1_ENC, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY);
Capstan capstan_2(T2_DIR, T2_PWM, T2_FLT, T2_CS, MUX_ADDR, T2_ENC, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY);
Capstan capstan_3(T3_DIR, T3_PWM, T3_FLT, T3_CS, MUX_ADDR, T3_ENC, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY);
S_K_Phi parameters(DEFAULT_S, DEFAULT_K, DEFAULT_PHI);
Hose hose(parameters, TENDON_DISTANCE, UPDATE_TIME);

void setup() {
    setup_communication();
    hose.add_capstan(&capstan_1);
    hose.add_capstan(&capstan_2);
    hose.add_capstan(&capstan_3);
    hose.init(false); // do not reset encoder zero positions
}

void loop() {
    process_command();
    hose.update();
    Serial.println(capstan_1.get_current());
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
        double s = atof(char_buffers[0]);
        double k = atof(char_buffers[1]);
        double phi = atof(char_buffers[2]);
        S_K_Phi parameters(s, k, phi);
        double duration = atof(char_buffers[3]);
        hose.set_parameters(parameters, duration);
    }
}


