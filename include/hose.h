#ifndef hose_h
#define hose_h

#include <Arduino.h>
#include <capstan.h>
#include <config.h>

class S_K_Phi;

// implements tendon based continuum kinematics
class Hose {
    public:
        Hose(double tendon_distance, uint16_t update_time);
        void add_capstan(Capstan *capstan);
        void init();
        S_K_Phi get_parameters_s_k_phi();
        void set_parameters_s_k_phi(double s, double k, double phi, double duration);
        void update();
    private:
        double _tendon_distance; // distance between backbone and tendons [mm]
        uint8_t _num_capstans; // number of capstans/tendons attached to hose
        double _s, _k, _phi; // kinematic parameters of hose
        double _s_increment, _k_increment, _phi_increment; // amounts to increment during interpolation [mm], [mm^-1], [radians]
        uint16_t _update_time; // time between setpoint updates during interpolation [ms]
        uint32_t _updates; // number of updates remaining in interpolation
        uint32_t _timer; // timer used to interpolate change in parameters [ms]
        Capstan *_capstans[NUM_CAPSTANS];
        void update_parameters();
};

class S_K_Phi {
    public:
        S_K_Phi(double s, double k, double phi) : parameters {s, k, phi} {}
        double operator[](int index) { return parameters[index]; } 
        double parameters[3];
};

#endif
