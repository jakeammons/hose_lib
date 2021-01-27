#include <Arduino.h>
#include <wire.h>
#include "config.h"
#include "capstan.h"

void process_command();
void setup_communication();

// capstan constructor sets starting encoder position as zero
Capstan capstan_1(T1_DIR, T1_PWM, T1_FLT, T1_CS, MUX_ADDR, T1_ENC, KP, KI, KD, CIRCUMFERENCE);
Capstan capstan_2(T2_DIR, T2_PWM, T2_FLT, T2_CS, MUX_ADDR, T2_ENC, KP, KI, KD, CIRCUMFERENCE);
Capstan capstan_3(T3_DIR, T3_PWM, T3_FLT, T3_CS, MUX_ADDR, T3_ENC, KP, KI, KD, CIRCUMFERENCE);

double s = 900; // arc length [mm]
double k = .002; // curvature (1/500) [mm^-1]
double phi = 0; // angle about z_0 axis [radians]
double R = 30; // distance between tendon and backbone [mm]
double l_1 = s * (1 - R * k * cos(phi)); // [mm]
double l_2 = s * (1 - R * k * cos((4 * PI / 3) - phi)); // [mm]
double l_3 = s * (1 - R * k * cos((2 * PI / 3) - phi)); // [mm]

// tendon retraction is currently positive and extension is currently negative
// this can be changed with a combination of hardware and firmware
double l_1_d = 900 - l_1;
double l_2_d = 900 - l_2;
double l_3_d = 900 - l_3;
double count = 0;
bool start = 0;
bool extend = 1;
unsigned long timer = 0;

void setup() {
    setup_communication();
    capstan_1.init();
    capstan_2.init();
    capstan_3.init();
}

void loop() {
    process_command();
    if (millis() > timer)
    {
        if (start && extend && count < 100)
        {
            capstan_1.set_length(l_1_d * count / 100);
            capstan_2.set_length(l_2_d * count / 100);
            capstan_3.set_length(l_3_d * count / 100);
            count++;
        }
        timer = millis() + 50;
    }
    capstan_1.update();
    capstan_2.update();
    capstan_3.update();

    Serial.print("Capstan 1: ");
    Serial.print(capstan_1.get_length());
    Serial.print(" Capstan 2: ");
    Serial.print(capstan_2.get_length());
    Serial.print(" Capstan 3: ");
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
        char command = Serial.read();
        if (command == 's') {
            start = 1;
        }
    }
}


