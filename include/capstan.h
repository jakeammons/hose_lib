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
        Capstan(uint8_t dir, uint8_t pwm, uint8_t flt, uint8_t cs, uint8_t mux, uint8_t enc, double kp, double ki, double kd);
        void init();
        double get_angle();
        void set_angle(double setpoint);
        void update();
    private:
        uint8_t dir_; // capstan motor driver direction pin
        uint8_t pwm_; // capstan motor driver pwm pin
        uint8_t flt_; // capstan motor driver fault pin
        uint8_t cs_; // capstan motor driver current sense pin
        uint8_t mux_; // capstan encoder i2c mux address
        uint8_t enc_; // capstan encoder i2c mux channel
        double previous_angle; // absolute angle of last reading
        double current_angle; // absolute angle of current reading
        int revolutions; // number of full revolutions relative to zero
        double setpoint_, input_, output_; // pid input/output
        double kp_ = 0, ki_ = 0, kd_ = 0; // pid gains
        AMS_5600 ams5600;
        PID pid;
        double calc_angle();
        void select_channel();
};

#endif
