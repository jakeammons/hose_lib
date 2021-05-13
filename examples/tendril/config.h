// capstan position control gains
#define KP 12.0
#define KI 0.0
#define KD 0.0

// capstan parameters
#define CIRCUMFERENCE 69.0 // [mm]
#define MAX_VELOCITY .01 // [mm/ms]

// encoder parameters
#define PPR 1200

// hose parameters
#define TENDON_DISTANCE 6.5 // distance between tendon and backbone [mm]

#define UPDATE_TIME 20.0 // time between interpolation updates [ms]

#define DEFAULT_S 400.0 // starting backbone length [mm]
#define DEFAULT_U 0.0 // starting u parameter
#define DEFAULT_V 0.0 // starting v parameter

#define DEFAULT_K .0000001 // starting curvature [mm^-1]
#define DEFAULT_PHI 0.0 // starting bending plane [radians]

// motor driver pins
#define M1_DIR 4
#define M1_PWM 5

#define M2_DIR 6
#define M2_PWM 7

#define M3_DIR 8
#define M3_PWM 9

// encoder pins
#define M1_ENA 20
#define M1_ENB 21

#define M2_ENA 18
#define M2_ENB 19

#define M3_ENA 2
#define M3_ENB 3
