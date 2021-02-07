#ifndef hose_h
#define hose_h

#include <Arduino.h>
#include <capstan.h>
#include <config.h>

struct S_K_Phi {
    S_K_Phi(double s_in, double k_in, double phi_in) : s(s_in), k(k_in), phi(phi_in) { }
    S_K_Phi(const S_K_Phi& in) : s(in.s), k(in.k), phi(in.phi) { }
    S_K_Phi() : s(0), k(0), phi(0) { }
    double s, k, phi;
};

// implements tendon based continuum kinematics
class Hose {
    public:
        Hose(S_K_Phi s_k_phi, double tendon_distance, uint16_t update_time);
        void add_capstan(Capstan *capstan);
        void init(bool reset_zero);
        S_K_Phi get_parameters();
        void set_parameters(S_K_Phi s_k_phi, double duration);
        void update();
    private:
        S_K_Phi _parameters; // kinematic parameters of hose
        double _tendon_distance; // distance between backbone and tendons [mm]
        uint8_t _num_capstans; // number of capstans/tendons attached to hose
        S_K_Phi _parameter_increments; // amounts to increment during interpolation [mm], [mm^-1], [radians]
        uint16_t _update_time; // time between setpoint updates during interpolation [ms]
        uint32_t _updates; // number of updates remaining in interpolation
        uint32_t _timer; // timer used to interpolate change in parameters [ms]
        Capstan *_capstans[NUM_CAPSTANS];
        void update_parameters();
};

#endif
