#ifndef capstan_h
#define capstan_h

#include <Arduino.h>
#include <PID_v1.h>
#include <AS5600.h>
#include <Wire.h>

// provides encoder, motor driver, and pid control functions
// setpoint and input are relative angle in signed degrees (-inf, inf)
// output is duty cycle in signed pwm values (-255, 255)
class Capstan {
    public:
        Capstan(uint8_t dir, uint8_t pwm, uint8_t flt, uint8_t cs, uint8_t mux, uint8_t enc, double kp, double ki, double kd, double circumference, double max_velocity);
        void init();
        double get_angle();
        void set_angle(double angle);
        double get_length();
        void set_length(double length);
        double get_current();
        void update();
    private:
        uint8_t _dir; // capstan motor driver direction pin
        uint8_t _pwm; // capstan motor driver pwm pin
        uint8_t _flt; // capstan motor driver fault pin
        uint8_t _cs; // capstan motor driver current sense pin
        uint8_t _mux; // capstan encoder i2c mux address
        uint8_t _enc; // capstan encoder i2c mux channel
        double _setpoint, _input, _output; // pid input/output
        double _kp = 0, _ki = 0, _kd = 0; // pid gains
        double _previous_angle; // absolute angle of last reading [deg]
        double _current_angle; // absolute angle of current reading [deg]
        int8_t _revolutions; // number of full revolutions relative to zero
        double _circumference; // capstan circumference [mm]
        double _max_velocity; // maximum velocity allowed for tendon movements [mm/ms]
        AMS_5600 ams5600;
        PID pid;
        double calc_angle();
        void select_channel();
};

#endif
