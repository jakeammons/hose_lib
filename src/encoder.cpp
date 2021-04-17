#include "encoder.h"

/////////////////////////////////////////////////////////
//                  MAGNETIC ENCODER                   //
/////////////////////////////////////////////////////////
MagneticEncoder::MagneticEncoder(uint8_t mux, uint8_t enc)
    : _mux(mux),
    _enc(enc),
    _previous_angle(0.0),
    _current_angle(0.0),
    _revolutions(0) { 
    }

MagneticEncoder::MagneticEncoder(const MagneticEncoder &other)
    : _mux(other._mux),
    _enc(other._enc),
    _previous_angle(other._previous_angle),
    _current_angle(other._current_angle),
    _revolutions(other._revolutions) { 
    }

// initializes magnetic encoder by either reading or writing home position
void MagneticEncoder::init(uint8_t id, bool reset_zero) {
    uint16_t encoder_zero;
    select_channel();
    if (reset_zero)
    {
        encoder_zero = ams5600.setStartPosition();
        EEPROM.put(id * 2, encoder_zero); // 2 bytes per encoder zero value
    }
    else
    {
        EEPROM.get(id * 2, encoder_zero);
        ams5600.setStartPosition(encoder_zero);
    }
}

// returns last known relative angle
// does not recalculate current relative angle
// relative to home position
// returns degrees
double MagneticEncoder::get_angle() {
    return _revolutions * 360.0 + _current_angle;
}

// gets angle from encoder
// converts absolute angle to relative angle
double MagneticEncoder::calc_angle()
{
    // get current value (0-4096) from encoder and convert to angle (0-360)
    select_channel();
    _current_angle = ams5600.getScaledAngle() * 360.0/4096.0;
    // clamp angle to between 0 and 360
    if (_current_angle < 0.0)
        _current_angle = 0.0;
    if (_current_angle > 360.0)
        _current_angle = 360.0;
    // check if angle has crossed zero and adjust revolution count
    if (_current_angle < 90.0 && _previous_angle > 270.0)
        _revolutions++;
    if (_current_angle > 270.0 && _previous_angle < 90.0)
        _revolutions--;
    _previous_angle = _current_angle;
    // return angle relative to zero
    return (_revolutions * 360.0) + _current_angle;
}

// write mux channel address to i2c bus
// all subsequent transmissions are sent to mux channel
void MagneticEncoder::select_channel() 
{
    Wire.beginTransmission(_mux);
    Wire.write(1 << _enc);
    Wire.endTransmission();  
}

/////////////////////////////////////////////////////////
//                 QUADRATURE ENCODER                  //
/////////////////////////////////////////////////////////
QuadratureEncoder::QuadratureEncoder(uint8_t enc_a, uint8_t enc_b, uint16_t ppr)
    : _enc_a(enc_a),
    _enc_b(enc_b),
    _ppr(ppr),
    encoder(_enc_a, _enc_b) { 
    }

QuadratureEncoder::QuadratureEncoder(const QuadratureEncoder &other)
    : _enc_a(other._enc_a),
    _enc_b(other._enc_b),
    _ppr(other._ppr),
    encoder(other.encoder) { 
    }

void QuadratureEncoder::init(uint8_t id, bool reset_zero) {
}

// returns last known relative angle
// relative to position at startup
// returns degrees
double QuadratureEncoder::get_angle() {
    return ((double) encoder.getEncoderCount() / _ppr) * 360.0;
}
