#ifndef encoder_h
#define encoder_h

#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>
#include <EEPROM.h>
#include <QuadratureEncoder.h>

class Encoder {
    public:
        virtual void init(uint8_t id, bool reset_zero) = 0;
        virtual double get_angle() = 0;
};

class MagneticEncoder : public Encoder {
    public:
        MagneticEncoder(uint8_t mux, uint8_t enc);
        MagneticEncoder(const MagneticEncoder &other);
        void init(uint8_t id, bool reset_zero);
        double get_angle();
    private:
        uint8_t _mux; // capstan encoder i2c mux address
        uint8_t _enc; // capstan encoder i2c mux channel
        double _previous_angle; // absolute angle of last reading [deg]
        double _current_angle; // absolute angle of current reading [deg]
        int8_t _revolutions; // number of full revolutions relative to zero
        AMS_5600 ams5600;
        double calc_angle();
        void select_channel();
};

class QuadratureEncoder : public Encoder {
    public:
        QuadratureEncoder(uint8_t enc_a, uint8_t enc_b, uint16_t ppr);
        QuadratureEncoder(const QuadratureEncoder &other);
        void init(uint8_t id, bool reset_zero);
        double get_angle();
    private:
        uint8_t _enc_a; // quadrature signal a
        uint8_t _enc_b; // quadrature signal b
        uint16_t _ppr; // pulses per revolutions
        Encoders encoder;
};

#endif
