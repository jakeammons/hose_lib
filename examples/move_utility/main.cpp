#include <Arduino.h>
#include <wire.h>
#include "capstan.h"
#include "config.h"
#include "kinematics.h"

void process_command();
void setup_communication();

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

Capstan *current;
uint8_t delta = 5;

void setup() {
    setup_communication();
    kinematics.add_capstan(capstan_1);
    kinematics.add_capstan(capstan_2);
    kinematics.add_capstan(capstan_3);
    kinematics.add_capstan(capstan_4);
    kinematics.add_capstan(capstan_5);
    kinematics.add_capstan(capstan_6);
    // reset encoder zero positions
    kinematics.init(true);
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

void process_command()
{
	if (Serial.available() > 0) {
		char command = Serial.read();
		if (command == '1') {
            current = &capstan_1;
        }
        else if (command == '2') {
            current = &capstan_2;
        }
        else if (command == '3') {
            current = &capstan_3;
        }
        else if (command == '4') {
            current = &capstan_4;
        }
        else if (command == '5') {
            current = &capstan_5;
        }
        else if (command == '6') {
            current = &capstan_6;
        }
        else if (command == 'e') {
            current->set_length(current->get_length() + delta);
		}
		else if (command == 'r') {
            current->set_length(current->get_length() - delta);
		}
		else if (command == '+') {
            delta++;
		}
		else if (command == '-') {
            delta--;
		}
	}
    Serial.print("Capstan 1: ");
    Serial.print(capstan_1.get_length());
    Serial.print(" Capstan 2: ");
    Serial.print(capstan_2.get_length());
    Serial.print(" Capstan 3: ");
    Serial.print(capstan_3.get_length());
    Serial.print(" Capstan 4: ");
    Serial.print(capstan_4.get_length());
    Serial.print(" Capstan 5: ");
    Serial.print(capstan_5.get_length());
    Serial.print(" Capstan 6: ");
    Serial.print(capstan_6.get_length());
    Serial.print(" delta: ");
    Serial.println(delta);
}
