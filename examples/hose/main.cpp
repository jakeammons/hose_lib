#include <Arduino.h>
#include <wire.h>
#include "config.h"
#include "kinematics.h"

void setup_communication();
void process_command();
void print_parameters();
void print_currents();

Encoder *encoder_1 = new MagneticEncoder(MUX_ADDR, M1_ENC);
Encoder *encoder_2 = new MagneticEncoder(MUX_ADDR, M2_ENC);
Encoder *encoder_3 = new MagneticEncoder(MUX_ADDR, M3_ENC);
Encoder *encoder_4 = new MagneticEncoder(MUX_ADDR, M4_ENC);
Encoder *encoder_5 = new MagneticEncoder(MUX_ADDR, M5_ENC);
Encoder *encoder_6 = new MagneticEncoder(MUX_ADDR, M6_ENC);
Capstan capstan_1(M1_DIR, M1_PWM, M1_FLT, M1_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_1);
Capstan capstan_2(M2_DIR, M2_PWM, M2_FLT, M2_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_2);
Capstan capstan_3(M3_DIR, M3_PWM, M3_FLT, M3_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_3);
Capstan capstan_4(M4_DIR, M4_PWM, M4_FLT, M4_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_4);
Capstan capstan_5(M5_DIR, M5_PWM, M5_FLT, M5_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_5);
Capstan capstan_6(M6_DIR, M6_PWM, M6_FLT, M6_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_6);
S_U_V parameters(DEFAULT_S, DEFAULT_U, DEFAULT_V);
Kinematics kinematics(parameters, TENDON_DISTANCE, UPDATE_TIME);

void setup() {
    setup_communication();
    kinematics.add_capstan(capstan_1);
    kinematics.add_capstan(capstan_2);
    kinematics.add_capstan(capstan_3);
    kinematics.add_capstan(capstan_4);
    kinematics.add_capstan(capstan_5);
    kinematics.add_capstan(capstan_6);
    // calculates current configuration and interpolates to home position
    // argument of false means don't reset encoder zero positions
    kinematics.init(false);
}

void loop() {
    process_command();
    kinematics.update();
}

void setup_communication() {
    // change pin 3 pwm frequency to 31kHz
    TCCR3B = (TCCR3B & B11111000) | B00000001;
    // change pin 6 pwm frequency to 31kHz
    TCCR4B = (TCCR4B & B11111000) | B00000001;
    // change pin 9 pwm frequency to 31kHz
    TCCR2B = (TCCR2B & B11111000) | B00000001;
    // change pin 12 pwm frequency to 31kHz
    TCCR1B = (TCCR1B & B11111000) | B00000001;
    // change pin 44, 45 pwm frequency to 31kHz
    TCCR5B = (TCCR5B & B11111000) | B00000001;

    // initialize i2c and UART
    Wire.begin();
    Serial.begin(9600);
}

void process_command() {
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
        double u = atof(char_buffers[1]);
        double v = atof(char_buffers[2]);
        S_U_V parameters(s, u, v);
        double duration = atof(char_buffers[3]);
        kinematics.set_parameters(parameters, duration);
    }
}

void print_parameters() {
    S_U_V parameters;
    kinematics.get_parameters(parameters);
    Serial.print("s: ");
    Serial.print(parameters.s);
    Serial.print(", u: ");
    Serial.print(parameters.u, 4);
    Serial.print(", v: ");
    Serial.print(parameters.v, 4);
}

void print_currents() {
    for (uint8_t i = 0; i < 6; i++) {
        uint16_t adc_value = kinematics.get_capstan(i)->get_current();
        double adc_voltage = adc_value * 5.0 / 1024.0;
        double current = adc_voltage / CS_CONVERSION + CS_OFFSET;
        Serial.print("motor ");
        Serial.print(i + 1);
        Serial.print(" current: ");
        Serial.print(current);
        Serial.print(", ");
    }
}
