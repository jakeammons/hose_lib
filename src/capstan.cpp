#include "capstan.h"

Capstan::Capstan(uint8_t dir, uint8_t pwm, uint8_t flt, uint8_t cs, uint8_t mux, uint8_t enc, double kp, double ki, double kd, double circumference, double max_velocity)
    : _dir(dir),
    _pwm(pwm),
    _flt(flt),
    _cs(cs),
    _mux(mux),
    _enc(enc),
    _setpoint(0),
    _input(0),
    _output(0),
    _kp(kp),
    _ki(ki),
    _kd(kd),
    _previous_angle(0),
    _current_angle(0),
    _revolutions(0),
    _circumference(circumference),
    _max_velocity(max_velocity),
    pid(&_input, &_output, &_setpoint, _kp, _ki, _kd, DIRECT) {
        pinMode(_dir, OUTPUT);
        pinMode(_pwm, OUTPUT);
        pinMode(_flt, INPUT);
        pinMode(_cs, INPUT);
    }

void Capstan::init(uint8_t id, bool reset_zero) {
    uint16_t encoder_zero;
    select_channel();
    if (reset_zero)
    {
        encoder_zero = ams5600.setStartPosition();
        EEPROM.put(id * 2, encoder_zero); // 2 bytes per encoder zero value
    }
    else
    {
        EEPROM.get(id * 2, encoder_zero);
        ams5600.setStartPosition(encoder_zero);
    }
    pid.SetOutputLimits(-255, 255); // gives both direction and magnitude with full pwm resolution
    pid.SetSampleTime(1); // 1 ms
    _input = calc_angle();
    _setpoint = _input;
    pid.SetMode(AUTOMATIC);
}

// returns last known relative angle
// does not recalculate current relative angle
// returns degrees
double Capstan::get_angle() {
    return (_revolutions * 360) + _current_angle;
}

// updates setpoint in terms of relative capstan angle
void Capstan::set_angle(double angle) {
    _setpoint = angle;
}

// returns tendon length using last know relative angle
// does not recalculate current relative angle
// returns mm
double Capstan::get_length() {
    return ((_revolutions * 360) + _current_angle) * (_circumference / 360);
}

// updates tendon length relative to zero position
// length in mm
void Capstan::set_length(double length) {
    _setpoint = (length * 360) / _circumference;
}

// returns motor driver current
// returns amps
double Capstan::get_current() {
    return (analogRead(_cs) * 100) + 5;
}

// computes pid control output and updates motor driver
void Capstan::update() {
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

// write mux channel address to i2c bus
// all subsequent transmissions are sent to mux channel
void Capstan::select_channel() 
{
    Wire.beginTransmission(_mux);
    Wire.write(1 << _enc);
    Wire.endTransmission();  
}
