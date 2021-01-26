#include "capstan.h"

Capstan::Capstan(uint8_t dir, uint8_t pwm, uint8_t flt, uint8_t cs, uint8_t mux, uint8_t enc, double kp, double ki, double kd)
    : dir_(dir),
    pwm_(pwm),
    flt_(flt),
    cs_(cs),
    mux_(mux),
    enc_(enc),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    pid(&input_, &output_, &setpoint_, kp_, ki_, kd_, DIRECT) {
        pinMode(dir_, OUTPUT);
        pinMode(pwm_, OUTPUT);
        pinMode(flt_, INPUT);
        pinMode(cs_, INPUT);
    }

void Capstan::init() {
    select_channel();
    ams5600.setStartPosition();
    pid.SetOutputLimits(-255, 255);
    input_ = calc_angle();
    setpoint_ = 0;
    pid.SetMode(AUTOMATIC);
}

// gets last known relative angle
// does not calculate current angle
double Capstan::get_angle() {
    return (revolutions * 360) + current_angle;
}

// updates setpoint
void Capstan::set_angle(double setpoint) {
    setpoint_ = setpoint;
}

// computes pid control output and updates motor driver
void Capstan::update() {
    input_ = calc_angle();
    pid.Compute();
    if (output_ > 0)
        digitalWrite(dir_, LOW);
    else
        digitalWrite(dir_, HIGH);
    analogWrite(pwm_, abs(output_));
    Serial.print("set: ");
    Serial.print(setpoint_);
    Serial.print(" in: ");
    Serial.print(input_);
    Serial.print(" out: ");
    Serial.print(output_);
    Serial.println();
}

// gets angle from encoder
// converts absolute angle to relative angle
double Capstan::calc_angle()
{
    // get current value (0-4096) from encoder and convert to angle (0-360)
    select_channel();
    current_angle = ams5600.getScaledAngle() * (double) 360/4096;
    // clamp angle to between 0 and 360
    if (current_angle < 0)
        current_angle = 0;
    if (current_angle > 360)
        current_angle = 360;
    // check if angle has crossed zero and adjust revolution count
    if (current_angle < 90 && previous_angle > 270)
        revolutions++;
    if (current_angle > 270 && previous_angle < 90)
        revolutions--;
    previous_angle = current_angle;
    // return angle relative to zero
    return (revolutions * 360) + current_angle;
}

void Capstan::select_channel() 
{
    // write mux channel address to bus
    Wire.beginTransmission(mux_);
    Wire.write(1 << enc_);
    Wire.endTransmission();  
}
