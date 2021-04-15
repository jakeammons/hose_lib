#include <Arduino.h>
#include <wire.h>
#include "config.h"
#include "kinematics.h"

void process_command();
void setup_communication();

Encoder *encoder_1 = new MagneticEncoder(MUX_ADDR, T1_ENC);
Encoder *encoder_2 = new MagneticEncoder(MUX_ADDR, T2_ENC);
Encoder *encoder_3 = new MagneticEncoder(MUX_ADDR, T3_ENC);
Capstan capstan_1(T1_DIR, T1_PWM, T1_FLT, T1_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_1);
Capstan capstan_2(T2_DIR, T2_PWM, T2_FLT, T2_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_2);
Capstan capstan_3(T3_DIR, T3_PWM, T3_FLT, T3_CS, KP, KI, KD, CIRCUMFERENCE, MAX_VELOCITY, DIRECT, encoder_3);
S_K_Phi parameters(DEFAULT_S, DEFAULT_K, DEFAULT_PHI);
Kinematics kinematics(parameters, TENDON_DISTANCE, UPDATE_TIME);

void setup() {
    setup_communication();
    kinematics.add_capstan(&capstan_1);
    kinematics.add_capstan(&capstan_2);
    kinematics.add_capstan(&capstan_3);
    // calculates current configuration and interpolates to home position
    // IMPORTANT: phi stays the same after interpolation
    // you may want to interpolate to new phi after robot returns home
    // argument of false means don't reset encoder zero positions
    kinematics.init(false);
}

void loop() {
    process_command();
    kinematics.update();
    S_K_Phi parameters;
    kinematics.get_parameters(parameters);
    Serial.print(parameters.s);
    Serial.print(",");
    Serial.print(parameters.k, 4);
    Serial.print(",");
    Serial.println(parameters.phi * 180 / M_PI);
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
        kinematics.set_parameters(parameters, duration);
    }
}


