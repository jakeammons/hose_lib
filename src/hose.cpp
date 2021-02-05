#include "hose.h"

Hose::Hose(double tendon_distance, uint16_t update_time)
    : _tendon_distance(tendon_distance),
    _num_capstans(0),
    _s(0),
    _k(0),
    _phi(0),
    _s_increment(0),
    _k_increment(0),
    _phi_increment(0),
    _update_time(update_time),
    _updates(0),
    _timer(0) { 
}

// add capstan/tendon to kinematic model and control
void Hose::add_capstan(Capstan *capstan) {
    _capstans[_num_capstans] = capstan;
    _num_capstans++;
}

// initializes interpolation targets and capstan positions
// reset_zero causes the current tendon lengths to be saved as "home"
void Hose::init(bool reset_zero) {
    // get current tendon lengths
    // calculate kinematic parameters from tendon lengths
    // set interpolation target to home configuration (straight)
    for (int i = 0; i < _num_capstans; i++)
        _capstans[i]->init(i, reset_zero);
    set_parameters_s_k_phi(740, .0000001, 0, 5000);
}

// gets tendon lengths and computes inverse kinematics
// returns s, k, and phi in mm, mm^-1, and radians
S_K_Phi Hose::get_parameters_s_k_phi() {
    S_K_Phi s_k_phi(0, 0, 0);
    return s_k_phi;
}

// updates target parameters
// interpolates parameter changes over set duration
// target_s in mm
// target_k in mm^-1
// target_phi in radians
// duration in ms
void Hose::set_parameters_s_k_phi(double s_target, double k_target, double phi_target, double duration) {
    _updates = duration / _update_time;
    _s_increment = (s_target - _s) / _updates;
    _k_increment = (k_target - _k) / _updates;
    _phi_increment = (phi_target - _phi) / _updates;
    _timer = millis();
}

// updates interpolation and tendon lengths
void Hose::update() {
    if (_updates && millis() > _timer) {
        update_parameters();
        _timer = millis() + _update_time;
    }
    for (int i = 0; i < _num_capstans; i++)
    {
        _capstans[i]->update();
    }
}

// updates tendon lengths by interpolating kinematic parameters over time
void Hose::update_parameters() {
    _s += _s_increment;
    _k += _k_increment;
    _phi += _phi_increment;
    for (int i = 0; i < _num_capstans; i++)
    {
        double angle = (360 - (i * (360 / _num_capstans))) * M_PI / 180;
        double tendon_length = _s * (1 - _tendon_distance * _k * cos(angle - _phi));
        _capstans[i]->set_length(tendon_length - _s);
        Serial.print(tendon_length);
        Serial.print(",");
    }
    Serial.println();
    _updates--;
}
