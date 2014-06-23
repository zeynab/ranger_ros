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
    void reset(float x = 0., float y = 0., float theta = 0.);
    void update(int l_enc, int r_enc);
    void compute();

    double get_x() {return x;}
    double get_y() {return y;}
    double get_th() {return th;}
    double get_dx() {return dx;}
    double get_dr() {return dr;}

    /** Returns the speed to apply on left and right motors for a 
    given (v, w) (in m.s^-1 and rad.s^-1).

    Uses an approximation that works for small rotation speeds.

    Returns (left speed, right speed) in m.s^-1
    **/
    std::pair<double, double> twist_to_motors(double v, double w);

private:

    int enc_left, enc_right;
    int left, right;
    int lmult, rmult;
    int prev_lencoder, prev_rencoder;
    double x, y, th;
    double dx, dr;
    struct timeval then;
    double base_width, ticks_meter;


};
