/** Compute odometry from encoders for the EPFL Ranger v2 robot
 **/
#include <cmath>
#include <cstddef> // NULL

#include "odometry.h"

using namespace std;

RangerOdometry::RangerOdometry():
    enc_left(std::numeric_limits<int>::max()),
    enc_right(std::numeric_limits<int>::max()),
    lmult(0), rmult(0),
    prev_lencoder(0), prev_rencoder(0),
    _x(0.), _y(0.), _th(0.), _dx(0.), _dr(0.)
{

    gettimeofday(&then, NULL);

    set_wheelbase(RANGER_WHEEL_BASE);
    set_wheelradius(RANGER_WHEEL_RADIUS);

}

void RangerOdometry::set_wheelbase(double wheelbase) {
    base_width = wheelbase;
}

void RangerOdometry::set_wheelradius(double radius) {
    ticks_meter = TICKS_360 / (2 * M_PI * radius);
}

void RangerOdometry::reset(double x, double y, double theta) {
    enc_left = std::numeric_limits<int>::max();        // wheel encoder readings
    enc_right = std::numeric_limits<int>::max();
    _x = x;                  // position in xy plane 
    _y = y;
    _th = theta;
    _dx = 0;                 // speeds in x/rotation
    _dr = 0;
    gettimeofday(&then, NULL);
}

void RangerOdometry::update(int l_enc, int r_enc) {

    // Left wheel
    if (l_enc < ENCODER_LOW_WRAP and prev_lencoder > ENCODER_HIGH_WRAP) {
        lmult = lmult + 1;
    }

    if (l_enc > ENCODER_HIGH_WRAP and prev_lencoder < ENCODER_LOW_WRAP) {
        lmult = lmult - 1;
    }

    int left = l_enc + lmult * (ENCODER_MAX - ENCODER_MIN);
    prev_lencoder = l_enc;

    // Right wheel
    if (r_enc < ENCODER_LOW_WRAP and prev_rencoder > ENCODER_HIGH_WRAP) {
        rmult = rmult + 1;
    }

    if (r_enc > ENCODER_HIGH_WRAP and prev_rencoder < ENCODER_LOW_WRAP) {
        rmult = rmult - 1;
    }

    int right = r_enc + rmult * (ENCODER_MAX - ENCODER_MIN);
    prev_rencoder = r_enc;

    compute(left, right);
}

/** Compute (x, y, theta, v, w) for the robot (in meters, seconds and radians).
**/
void RangerOdometry::compute(int left, int right) {
    struct timeval now;
    gettimeofday(&now, NULL);

    long seconds  = now.tv_sec  - then.tv_sec;
    long useconds = now.tv_usec - then.tv_usec;

    double elapsed = seconds + useconds/1000000.0;

    then = now;

    double d_left, d_right;

    if (enc_left == numeric_limits<int>::max()) {
        d_left = 0.;
        d_right = 0.;
    }
    else {
        d_left = (left - enc_left) / ticks_meter;
        d_right = (right - enc_right) / ticks_meter;
    }
    enc_left = left;
    enc_right = right;

    // distance traveled is the average of the two wheels 
    double d = ( d_left + d_right ) / 2;
    // this approximation works (in radians) for small angles
    double delta_th = ( d_right - d_left ) / base_width;

    // calculate velocities
    _dx = d / elapsed;
    _dr = delta_th / elapsed;

    if (d != 0) {
        // calculate distance traveled in x and y
        double delta_x = cos( delta_th ) * d;
        double delta_y = -sin( delta_th ) * d;
        // calculate the final position of the robot
        _x = _x + ( cos( _th ) * delta_x - sin( _th ) * delta_y );
        _y = _y + ( sin( _th ) * delta_x + cos( _th ) * delta_y );
    }

    if (delta_th != 0) {
        _th = _th + delta_th;
    }
}

/** Returns the speed to apply on left and right motors for a 
given (v, w) (in m.s^-1 and rad.s^-1).

Uses an approximation that works for small rotation speeds.

Returns (left speed, right speed) in m.s^-1
**/
pair<double, double> RangerOdometry::twist_to_motors(double v, double w) const {
    // dx = (l + r) / 2
    // dr = (r - l) / w
    double left = v + w * base_width / 2;
    double right = v - w * base_width / 2;

    return make_pair(left, right);
}

