#include "capstan.h"

Capstan::Capstan(uint8_t dir, uint8_t pwm, uint8_t flt, uint8_t cs, double kp, double ki, double kd, double circumference, double max_velocity, int direction, Encoder *enc)
    : _dir(dir),
    _pwm(pwm),
    _flt(flt),
    _cs(cs),
    _setpoint(0.0),
    _input(0.0),
    _output(0.0),
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _circumference(circumference),
    _max_velocity(max_velocity),
    encoder(enc),
    pid(&_input, &_output, &_setpoint, _kp, _ki, _kd, direction) {
        pinMode(_dir, OUTPUT);
        pinMode(_pwm, OUTPUT);
        pinMode(_flt, INPUT);
        pinMode(_cs, INPUT);
    }

void Capstan::init(uint8_t id, bool reset_zero) {
    encoder->init(id, reset_zero);
    pid.SetOutputLimits(-255.0, 255.0); // gives both direction and magnitude with full pwm resolution
    pid.SetSampleTime(1.0); // 1 ms
    _input = encoder->get_angle();
    _setpoint = _input;
    pid.SetMode(AUTOMATIC);
}

// updates setpoint in terms of relative capstan angle
void Capstan::set_angle(double angle) {
    _setpoint = angle;
}

// returns tendon length using last know relative angle
// does not recalculate current relative angle
// returns mm
double Capstan::get_length() {
    return encoder->get_angle() * (_circumference / 360.0);
}

// updates tendon length relative to zero position
// length in mm
void Capstan::set_length(double length) {
    _setpoint = (length * 360.0) / _circumference;
}

// returns motor driver current
// takes 100 samples from motor driver current sensor and returns max reading
// max reading seems to be less susceptible to noise than average reading
// returns amps
double Capstan::get_current() {
    uint16_t reading = analogRead(_cs);
    uint16_t max = reading;
    for (uint8_t i = 0; i < 100; i++)
    {
        reading = analogRead(_cs);
        if (reading > max)
            max = reading;
    }
    return (max * 5.0 / 1024.0) * 100.0 + 5.0;
}

// computes pid control output and updates motor driver
void Capstan::update() {
    _input = encoder->get_angle();
    pid.Compute();
    if (_output > 0.0)
        digitalWrite(_dir, LOW);
    else
        digitalWrite(_dir, HIGH);
    analogWrite(_pwm, abs(_output));
}
