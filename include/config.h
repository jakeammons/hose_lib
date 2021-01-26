// capstan position control gains
#define KP 12
#define KI 0
#define KD 0

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
