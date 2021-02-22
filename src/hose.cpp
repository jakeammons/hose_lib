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
    if (_num_capstans < MAX_CAPSTANS)
    {
        _capstans[_num_capstans] = capstan;
        _num_capstans++;
    }
}

// initializes capstan positions
// calculates current configuration and interpolates to home
// after interpolation, curvature changes to straight, but phi stays the same
// reset_zero causes the current tendon lengths to be saved as "home"
// only interpolates to home if not in "reset zero" mode
void Hose::init(bool reset_zero) {
    for (uint8_t i = 0; i < _num_capstans; i++)
        _capstans[i]->init(i, reset_zero);
    if (!reset_zero)
    {
        _parameters = get_parameters();
        S_K_Phi parameters(_parameters.s, .0000001, _parameters.phi);
        set_parameters(parameters, 5000);
    }
}

// gets tendon lengths and computes kinematic parameters
// returns s, k, and phi in mm, mm^-1, and radians
S_K_Phi Hose::get_parameters() {
    double tendon_lengths[_num_capstans];
    for (uint8_t i = 0; i < _num_capstans; i++)
        tendon_lengths[i] = _parameters.s + _capstans[i]->get_length();
    // s = (sum of tendon lengths) / (number of tendons)
    double s = 0;
    for (uint8_t i = 0; i < _num_capstans; i++)
        s += tendon_lengths[i];
    s /= _num_capstans;
    // TODO: generalize k and phi calculations to any number of tendons
    // k = average tendon curvature
    double l_1 = tendon_lengths[0];
    double l_2 = tendon_lengths[1];
    double l_3 = tendon_lengths[2];
    double k = 2 * sqrt(l_1 * l_1 + l_2 * l_2 + l_3 * l_3 - l_1 * l_2 - l_2 * l_3 - l_1 * l_3);
    k /= _tendon_distance * (l_1 + l_2 + l_3);
    // phi = ratio of bend towards x to bend towards y
    double x = -l_1 - l_2 * -0.5 - l_3 * -0.5;
    double y = -l_2 * sqrt(3)/2 - l_3 * -sqrt(3)/2;
    double phi = atan2(y, x);
    S_K_Phi parameters(s, k, phi);
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
        _timer = millis() + _update_time;
        update_parameters();
    }
    for (uint8_t i = 0; i < _num_capstans; i++)
        _capstans[i]->update();
}

// updates tendon lengths by interpolating kinematic parameters over time
void Hose::update_parameters() {
    _parameters.s += _parameter_increments.s;
    _parameters.k += _parameter_increments.k;
    _parameters.phi += _parameter_increments.phi;
    for (uint8_t i = 0; i < _num_capstans; i++)
    {
        double angle = ((i * (360 / _num_capstans)) - 360) * M_PI / 180;
        double tendon_length = _parameters.s * (1 - _tendon_distance * _parameters.k * cos(angle - _parameters.phi));
        _capstans[i]->set_length(tendon_length - _parameters.s);
    }
    _updates--;
}
