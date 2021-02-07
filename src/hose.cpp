#include "hose.h"

Hose::Hose(S_K_Phi parameters, double tendon_distance, uint16_t update_time)
    : _parameters(parameters),
    _tendon_distance(tendon_distance),
    _num_capstans(0),
    _parameter_increments(0, 0, 0),
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
    // TODO: get current tendon lengths
    // TODO: calculate kinematic parameters from tendon lengths
    // TODO: set interpolation target to home configuration (straight)
    for (int i = 0; i < _num_capstans; i++)
        _capstans[i]->init(i, reset_zero);
}

// gets tendon lengths and computes inverse kinematics
// returns s, k, and phi in mm, mm^-1, and radians
S_K_Phi Hose::get_parameters() {
    S_K_Phi parameters(0, 0, 0);
    return parameters;
}

// updates target parameters
// interpolates parameter changes over set duration
// parameters s, k, and phi in mm, mm^-1, and radians
// duration in ms
void Hose::set_parameters(S_K_Phi parameters, double duration) {
    _updates = duration / _update_time;
    _parameter_increments.s = (parameters.s - _parameters.s) / _updates;
    _parameter_increments.k = (parameters.k - _parameters.k) / _updates;
    _parameter_increments.phi = (parameters.phi - _parameters.phi) / _updates;
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
    _parameters.s += _parameter_increments.s;
    _parameters.k += _parameter_increments.k;
    _parameters.phi += _parameter_increments.phi;
    for (int i = 0; i < _num_capstans; i++)
    {
        double angle = ((i * (360 / _num_capstans)) - 360) * M_PI / 180;
        double tendon_length = _parameters.s * (1 - _tendon_distance * _parameters.k * cos(angle - _parameters.phi));
        _capstans[i]->set_length(tendon_length - _parameters.s);
        // Serial.print(tendon_length);
        // Serial.print(",");
    }
    // Serial.println();
    _updates--;
}
