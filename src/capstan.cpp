#include "capstan.h"

Capstan::Capstan(uint8_t dir, uint8_t pwm, uint8_t flt, uint8_t cs, uint8_t mux, uint8_t enc, double kp, double ki, double kd, double circumference, uint16_t update_time, double max_velocity)
    : _dir(dir),
    _pwm(pwm),
    _flt(flt),
    _cs(cs),
    _mux(mux),
    _enc(enc),
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _circumference(circumference),
    _update_time(update_time),
    _max_velocity(max_velocity),
    _previous_angle(0),
    _current_angle(0),
    _revolutions(0),
    _updates(0),
    _update_length(0),
    _timer(0),
    pid(&_input, &_output, &_setpoint, _kp, _ki, _kd, DIRECT) {
        pinMode(_dir, OUTPUT);
        pinMode(_pwm, OUTPUT);
        pinMode(_flt, INPUT);
        pinMode(_cs, INPUT);
    }

void Capstan::init() {
    select_channel();
    pid.SetOutputLimits(-255, 255); // gives both direction and magnitude with full pwm resolution
    pid.SetSampleTime(1); // 1 ms
    _input = calc_angle();
    _setpoint = _input;
    set_length(0, 5000);
    pid.SetMode(AUTOMATIC);
}

// returns last known relative angle
// does not recalculate current relative angle
double Capstan::get_angle() {
    return (_revolutions * 360) + _current_angle;
}

// updates setpoint in terms of relative capstan angle
void Capstan::set_angle(double angle) {
    _setpoint = angle;
}

// returns tendon length using last know relative angle
// does not recalculate current relative angle
double Capstan::get_length() {
    return ((_revolutions * 360) + _current_angle) * (_circumference / 360);
}

// updates target tendon length relative to zero position
// interpolates tendon length changes over set duration
// target_length in mm
// duration in ms
void Capstan::set_length(double target_length, uint32_t duration) {
    double delta = target_length - get_length();
    double velocity = delta / duration; // [mm/ms]
    // TODO account for negative velocities
    /*
    if (velocity > _max_velocity) {
        velocity = _max_velocity;
        duration = target_length / velocity;
    }
    */
    _updates = duration / _update_time;
    _update_length = delta / _updates;
    _timer = millis();
}

double Capstan::get_current() {
    return analogRead(_cs) * 100 + 5;
}

// computes pid control output and updates motor driver
void Capstan::update() {
    if (_updates && millis() > _timer) {
        update_length();
        _timer = millis() + _update_time;
    }
    _input = calc_angle();
    pid.Compute();
    if (_output > 0)
        digitalWrite(_dir, LOW);
    else
        digitalWrite(_dir, HIGH);
    analogWrite(_pwm, abs(_output));
}

// gets angle from encoder
// converts absolute angle to relative angle
double Capstan::calc_angle()
{
    // get current value (0-4096) from encoder and convert to angle (0-360)
    select_channel();
    _current_angle = ams5600.getScaledAngle() * (double) 360/4096;
    // clamp angle to between 0 and 360
    if (_current_angle < 0)
        _current_angle = 0;
    if (_current_angle > 360)
        _current_angle = 360;
    // check if angle has crossed zero and adjust revolution count
    if (_current_angle < 90 && _previous_angle > 270)
        _revolutions++;
    if (_current_angle > 270 && _previous_angle < 90)
        _revolutions--;
    _previous_angle = _current_angle;
    // return angle relative to zero
    return (_revolutions * 360) + _current_angle;
}

void Capstan::select_channel() 
{
    // write mux channel address to bus
    Wire.beginTransmission(_mux);
    Wire.write(1 << _enc);
    Wire.endTransmission();  
}

// updates setpoint by interpolating relative tendon length over time
void Capstan::update_length() {
    _setpoint += (_update_length * 360) / _circumference;
    _updates--;
}
