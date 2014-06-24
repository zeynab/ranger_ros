/** Compute odometry from encoders for the EPFL Ranger v2 robot
 **/

#include <sys/time.h>
#include <limits>
#include <utility> // std::pair

const double RANGER_WHEEL_RADIUS = 0.053; //m
const double RANGER_WHEEL_BASE = 0.301; //m

const int TICKS_360 = 43950; //The number of wheel encoder ticks per complete wheel turn

const int ENCODER_MIN = -32768;
const int ENCODER_MAX = 32768;

const int ENCODER_LOW_WRAP = (ENCODER_MAX - ENCODER_MIN) * 0.3 + ENCODER_MIN;
const int ENCODER_HIGH_WRAP = (ENCODER_MAX - ENCODER_MIN) * 0.7 + ENCODER_MIN;

class RangerOdometry {

public:
    RangerOdometry();
    void set_wheelbase(double wheelbase);
    void set_wheelradius(double radius);
    void reset(double x = 0., double y = 0., double theta = 0.);
    void update(int l_enc, int r_enc);

    double get_x() const {return _x;}
    double get_y() const {return _y;}
    double get_th() const {return _th;}
    double get_dx() const {return _dx;}
    double get_dr() const {return _dr;}

    /** Returns the speed to apply on left and right motors for a 
    given (v, w) (in m.s^-1 and rad.s^-1).

    Uses an approximation that works for small rotation speeds.

    Returns (left speed, right speed) in m.s^-1
    **/
    std::pair<double, double> twist_to_motors(double v, double w) const;

private:

    int enc_left, enc_right;
    int lmult, rmult;
    int prev_lencoder, prev_rencoder;
    double _x, _y, _th;
    double _dx, _dr;
    struct timeval then;
    double base_width, ticks_meter;

    void compute(int left, int right);

};
