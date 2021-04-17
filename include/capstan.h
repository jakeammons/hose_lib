#ifndef capstan_h
#define capstan_h

#include <Arduino.h>
#include <PID_v1.h>
#include <AS5600.h>
#include <Wire.h>
#include <EEPROM.h>
#include "encoder.h"

// provides encoder, motor driver, and pid control functions
// setpoint and input are relative angle in signed degrees (-inf, inf)
// output is duty cycle in signed pwm values (-255, 255)
class Capstan {
    public:
        Capstan(uint8_t dir, uint8_t pwm, uint8_t flt, uint8_t cs, double kp, double ki, double kd, double circumference, double max_velocity, int direction, Encoder *enc);
        void init(uint8_t id, bool reset_zero);
        double get_angle();
        void set_angle(double angle);
        double get_length();
        void set_length(double length);
        double get_current();
        void update();
    private:
        uint8_t _id; // capstan id
        uint8_t _dir; // motor driver direction pin
        uint8_t _pwm; // motor driver pwm pin
        uint8_t _flt; // motor driver fault pin
        uint8_t _cs; // current sense pin
        double _setpoint, _input, _output; // pid input/output
        double _kp, _ki, _kd; // pid gains
        double _circumference; // capstan circumference [mm]
        double _max_velocity; // maximum velocity allowed for tendon movements [mm/ms]
        Encoder *encoder;
        PID pid;
        void check_faults();
};

#endif
