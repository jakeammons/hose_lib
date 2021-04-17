#include "kinematics.h"

Kinematics::Kinematics(S_U_V parameters, double tendon_distance, uint16_t update_time)
    : _mode(S_U_V_MODE),
    _s_u_v_parameters(parameters),
    _s_k_phi_parameters(0.0, 0.0, 0.0),
    _tendon_distance(tendon_distance),
    _num_capstans(0),
    _s_u_v_parameter_increments(0.0, 0.0, 0.0),
    _s_k_phi_parameter_increments(0.0, 0.0, 0.0),
    _update_time(update_time),
    _updates(0),
    _timer(0) { 
    }

Kinematics::Kinematics(S_K_Phi parameters, double tendon_distance, uint16_t update_time)
    : _mode(S_K_PHI_MODE),
    _s_u_v_parameters(0.0, 0.0, 0.0),
    _s_k_phi_parameters(parameters),
    _tendon_distance(tendon_distance),
    _num_capstans(0),
    _s_u_v_parameter_increments(0.0, 0.0, 0.0),
    _s_k_phi_parameter_increments(0.0, 0.0, 0.0),
    _update_time(update_time),
    _updates(0),
    _timer(0) { 
    }

// add capstan/tendon to kinematic model and control
void Kinematics::add_capstan(Capstan *capstan) {
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
void Kinematics::init(bool reset_zero) {
    for (uint8_t i = 0; i < _num_capstans; i++)
        _capstans[i]->init(i, reset_zero);
    if (!reset_zero)
    {
        if (_mode == S_U_V_MODE)
        {
                get_parameters(_s_u_v_parameters);
                S_U_V parameters(_s_u_v_parameters.s, 0.0, 0.0);
                set_parameters(parameters, 5000.0);
        }
        else if (_mode == S_K_PHI_MODE)
        {
                get_parameters(_s_k_phi_parameters);
                S_K_Phi parameters(_s_k_phi_parameters.s, .0000001, _s_k_phi_parameters.phi);
                set_parameters(parameters, 5000.0);
        }
    }
}

// gets tendon lengths and computes kinematic parameters
// returns s [mm], u, and v
void Kinematics::get_parameters(S_U_V &parameters) {
    double tendon_lengths[_num_capstans];
    for (uint8_t i = 0; i < _num_capstans; i++)
        tendon_lengths[i] = _s_u_v_parameters.s + _capstans[i]->get_length();
    // s = (sum of tendon lengths) / (number of tendons)
    double s = 0;
    for (uint8_t i = 0; i < _num_capstans; i++)
        s += tendon_lengths[i];
    s /= _num_capstans;
    // TODO: generalize u and v calculations to any number of tendons
    double l_1 = tendon_lengths[0];
    double l_2 = tendon_lengths[1];
    double l_3 = tendon_lengths[2];
    double u = 0;
    u = (l_2 - l_3) / (sqrt(3) * _tendon_distance);
    double v = 0;
    v = (s - l_1) / _tendon_distance;
    parameters.s = s;
    parameters.u = u;
    parameters.v = v;
}

// gets tendon lengths and computes kinematic parameters
// returns s, k, and phi in mm, mm^-1, and radians
void Kinematics::get_parameters(S_K_Phi &parameters) {
    double tendon_lengths[_num_capstans];
    for (uint8_t i = 0; i < _num_capstans; i++)
        tendon_lengths[i] = _s_k_phi_parameters.s + _capstans[i]->get_length();
    // s = (sum of tendon lengths) / (number of tendons)
    double s = 0;
    for (uint8_t i = 0; i < _num_capstans; i++)
        s += tendon_lengths[i];
    s /= _num_capstans;
    // TODO: generalize k and phi calculations to any number of tendons
    double l_1 = tendon_lengths[0];
    double l_2 = tendon_lengths[1];
    double l_3 = tendon_lengths[2];
    // k = average tendon curvature
    double k = 2 * sqrt(l_1 * l_1 + l_2 * l_2 + l_3 * l_3 - l_1 * l_2 - l_2 * l_3 - l_1 * l_3);
    k /= _tendon_distance * (l_1 + l_2 + l_3);
    // phi = ratio of bend towards x to bend towards y
    double x = -l_1 - l_2 * -0.5 - l_3 * -0.5;
    double y = -l_2 * sqrt(3.0) / 2.0 - l_3 * -sqrt(3.0) / 2.0;
    double phi = atan2(y, x);
    parameters.s = s;
    parameters.k = k;
    parameters.phi = phi;
}

// updates target parameters
// interpolates parameter changes over set duration
// parameters s [mm], u, and v
// duration in ms
void Kinematics::set_parameters(S_U_V parameters, double duration) {
    _mode = S_U_V_MODE;
    _updates = duration / _update_time;
    _s_u_v_parameter_increments.s = (parameters.s - _s_u_v_parameters.s) / _updates;
    _s_u_v_parameter_increments.u = (parameters.u - _s_u_v_parameters.u) / _updates;
    _s_u_v_parameter_increments.v = (parameters.v - _s_u_v_parameters.v) / _updates;
    _timer = millis();
}

// updates target parameters
// interpolates parameter changes over set duration
// parameters s, k, and phi in mm, mm^-1, and radians
// duration in ms
void Kinematics::set_parameters(S_K_Phi parameters, double duration) {
    _mode = S_K_PHI_MODE;
    _updates = duration / _update_time;
    _s_k_phi_parameter_increments.s = (parameters.s - _s_k_phi_parameters.s) / _updates;
    _s_k_phi_parameter_increments.k = (parameters.k - _s_k_phi_parameters.k) / _updates;
    _s_k_phi_parameter_increments.phi = (parameters.phi - _s_k_phi_parameters.phi) / _updates;
    _timer = millis();
}

// updates interpolation and tendon lengths
void Kinematics::update() {
    if (_updates && millis() > _timer) {
        _timer = millis() + _update_time;
        update_parameters();
    }
    for (uint8_t i = 0; i < _num_capstans; i++)
        _capstans[i]->update();
}

// updates tendon lengths by interpolating kinematic parameters over time
void Kinematics::update_parameters() {
    if (_mode == S_U_V_MODE)
    {
            _s_u_v_parameters.s += _s_u_v_parameter_increments.s;
            _s_u_v_parameters.u += _s_u_v_parameter_increments.u;
            _s_u_v_parameters.v += _s_u_v_parameter_increments.v;
            double l[3];
            l[0] = _s_u_v_parameters.s - _tendon_distance * _s_u_v_parameters.v;
            l[1] = _s_u_v_parameters.s + .5 * _tendon_distance * (_s_u_v_parameters.v + sqrt(3) * _s_u_v_parameters.u);
            l[2] = _s_u_v_parameters.s + .5 * _tendon_distance * (_s_u_v_parameters.v - sqrt(3) * _s_u_v_parameters.u);
            for (uint8_t i = 0; i < 3; i++)
                _capstans[i]->set_length(l[i] - _s_u_v_parameters.s);
    }
    else if (_mode == S_K_PHI_MODE)
    {
            _s_k_phi_parameters.s += _s_k_phi_parameter_increments.s;
            _s_k_phi_parameters.k += _s_k_phi_parameter_increments.k;
            _s_k_phi_parameters.phi += _s_k_phi_parameter_increments.phi;
            for (uint8_t i = 0; i < _num_capstans; i++)
            {
                double angle = ((i * (360.0 / _num_capstans)) - 360.0) * M_PI / 180.0;
                double tendon_length = _s_k_phi_parameters.s * (1 - _tendon_distance * _s_k_phi_parameters.k * cos(angle - _s_k_phi_parameters.phi));
                _capstans[i]->set_length(tendon_length - _s_k_phi_parameters.s);
            }
    }
    _updates--;
}
