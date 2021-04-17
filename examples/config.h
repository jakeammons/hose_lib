// capstan position control gains
#define KP 12.0
#define KI 0.0
#define KD 0.0

// capstan parameters
#define CIRCUMFERENCE 314.0 // [mm]
#define MAX_VELOCITY .01 // [mm/ms]

// hose parameters
#define NUM_CAPSTANS 3 // number of capstans/tendons connected to hose
#define TENDON_DISTANCE 30.0 // distance between tendon and backbone [mm]
#define UPDATE_TIME 20.0 // time between interpolation updates [ms]
#define DEFAULT_S 825.0 // starting backbone length [mm]
#define DEFAULT_U 0.0 // starting u parameter
#define DEFAULT_V 0.0 // starting v parameter
#define DEFAULT_K .0000001 // starting curvature [mm^-1]
#define DEFAULT_PHI 0.0 // starting bending plane [radians]

// motor driver pins
#define T2_DIR 2
#define T2_PWM 3
#define T2_FLT 5
#define T2_CS  A2

#define T1_DIR 6
#define T1_PWM 7
#define T1_FLT 9
#define T1_CS  A1

#define T3_DIR 10
#define T3_PWM 11
#define T3_FLT 13
#define T3_CS  A0

// i2c mux channels
#define T1_ENC 0
#define T3_ENC 1
#define T2_ENC 2

// i2c mux address
#define MUX_ADDR 0x70
