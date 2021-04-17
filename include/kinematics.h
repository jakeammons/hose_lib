#ifndef kinematics_h
#define kinematics_h

#include <Arduino.h>
#include <capstan.h>

#define MAX_CAPSTANS 9

enum Parameter_Mode {
    S_U_V_MODE,
    S_K_PHI_MODE
};

struct S_U_V {
    S_U_V(double s_in, double u_in, double v_in) : s(s_in), u(u_in), v(v_in) { }
    S_U_V(const S_U_V &in) : s(in.s), u(in.u), v(in.v) { }
    S_U_V() : s(0.0), u(0.0), v(0.0) { }
    double s, u, v;
};

struct S_K_Phi {
    S_K_Phi(double s_in, double k_in, double phi_in) : s(s_in), k(k_in), phi(phi_in) { }
    S_K_Phi(const S_K_Phi &in) : s(in.s), k(in.k), phi(in.phi) { }
    S_K_Phi() : s(0.0), k(0.0), phi(0.0) { }
    double s, k, phi;
};

// implements tendon based continuum kinematics
class Kinematics {
    public:
        Kinematics(const S_U_V &parameters, double tendon_distance, uint16_t update_time);
        Kinematics(const S_K_Phi &parameters, double tendon_distance, uint16_t update_time);
        void add_capstan(Capstan &capstan);
        Capstan *get_capstan(uint8_t id);
        void init(bool reset_zero);
        void get_parameters(S_U_V &parameters);
        void get_parameters(S_K_Phi &parameters);
        void set_parameters(const S_U_V &parameters, double duration);
        void set_parameters(const S_K_Phi &parameters, double duration);
        void update();
    private:
        Parameter_Mode _mode; // current set of parameters used for updating tendon lengths
        S_U_V _s_u_v_parameters; // kinematic parameters of hose in terms of s, u, v
        S_K_Phi _s_k_phi_parameters; // kinematic parameters of hose in terms of s, k, phi
        const double _tendon_distance; // distance between backbone and tendons [mm]
        uint8_t _num_capstans; // number of capstans/tendons attached to hose
        S_U_V _s_u_v_parameter_increments; // amounts to increment during interpolation in terms of s, u, v [mm], [mm^-1], [radians]
        S_K_Phi _s_k_phi_parameter_increments; // amounts to increment during interpolation in terms of s, k, phi [mm], [mm^-1], [radians]
        uint16_t _update_time; // time between setpoint updates during interpolation [ms]
        uint32_t _updates; // number of updates remaining in interpolation
        uint32_t _timer; // timer used to interpolate change in parameters [ms]
        Capstan *_capstans[MAX_CAPSTANS];
        void update_parameters();
};

#endif
