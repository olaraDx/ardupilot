#include "AC_AttitudeControl_Multi.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AP_Scheduler/AP_Scheduler.h>
// #include <iostream>
// #include <fstream>
// #include <ctime>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Corrects in proportion to the difference between the desired roll rate vs actual roll rate
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_RLL_PDMX
    // @DisplayName: Roll axis rate controller PD sum maximum
    // @Description: Roll axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_RLL_D_FF
    // @DisplayName: Roll Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_RLL_NTF
    // @DisplayName: Roll Target notch filter index
    // @Description: Roll Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_RLL_NEF
    // @DisplayName: Roll Error notch filter index
    // @Description: Roll Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Corrects in proportion to the difference between the desired pitch rate vs actual pitch rate output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_PIT_PDMX
    // @DisplayName: Pitch axis rate controller PD sum maximum
    // @Description: Pitch axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_PIT_D_FF
    // @DisplayName: Pitch Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_PIT_NTF
    // @DisplayName: Pitch Target notch filter index
    // @Description: Pitch Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: RAT_PIT_NEF
    // @DisplayName: Pitch Error notch filter index
    // @Description: Pitch Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Corrects in proportion to the difference between the desired yaw rate vs actual yaw rate
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: RAT_YAW_PDMX
    // @DisplayName: Yaw axis rate controller PD sum maximum
    // @Description: Yaw axis rate controller PD sum maximum.  The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: RAT_YAW_D_FF
    // @DisplayName: Yaw Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.02
    // @Increment: 0.0001
    // @User: Advanced

    // @Param: RAT_YAW_NTF
    // @DisplayName: Yaw Target notch filter index
    // @Description: Yaw Target notch filter index
    // @Range: 1 8
    // @Units: Hz
    // @User: Advanced

    // @Param: RAT_YAW_NEF
    // @DisplayName: Yaw Error notch filter index
    // @Description: Yaw Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    // @Param: THR_G_BOOST
    // @DisplayName: Throttle-gain boost
    // @Description: Throttle-gain boost ratio. A value of 0 means no boosting is applied, a value of 1 means full boosting is applied. Describes the ratio increase that is applied to angle P and PD on pitch and roll.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("THR_G_BOOST", 7, AC_AttitudeControl_Multi, _throttle_gain_boost, 0.0f),

    AP_GROUPEND
};

AC_AttitudeControl_Multi::AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_MultiCopter &aparm, AP_MotorsMulticopter& motors) :
    AC_AttitudeControl(ahrs, aparm, motors),
    _motors_multi(motors)
{
    AP_Param::setup_object_defaults(this, var_info);

#if AP_FILTER_ENABLED
    set_notch_sample_rate(AP::scheduler().get_loop_rate_hz());
#endif
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));
}

void AC_AttitudeControl_Multi::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
    float cos_tilt_target = cosf(_thrust_angle);
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_gain_boost - boost angle_p/pd each cycle on high throttle slew
void AC_AttitudeControl_Multi::update_throttle_gain_boost()
{
    // Boost PD and Angle P on very rapid throttle changes
    if (_motors.get_throttle_slew_rate() > AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH) {
        const float pd_boost = constrain_float(_throttle_gain_boost + 1.0f, 1.0, 2.0);
        set_PD_scale_mult(Vector3f(pd_boost, pd_boost, 1.0f));

        const float angle_p_boost = constrain_float((_throttle_gain_boost + 1.0f) * (_throttle_gain_boost + 1.0f), 1.0, 4.0);
        set_angle_P_scale_mult(Vector3f(angle_p_boost, angle_p_boost, 1.0f));
    }
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);

        // if the mix is still higher than that being used, reset immediately
        const float throttle_hover = _motors.get_throttle_hover();
        const float throttle_in = _motors.get_throttle();
        const float throttle_out = MAX(_motors.get_throttle_out(), throttle_in);
        float mix_used;
        // since throttle_out >= throttle_in at this point we don't need to check throttle_in < throttle_hover
        if (throttle_out < throttle_hover) {
            mix_used = (throttle_out - throttle_in) / (throttle_hover - throttle_in);
        } else {
            mix_used = throttle_out / throttle_hover;
        }

        _throttle_rpy_mix = MIN(_throttle_rpy_mix, MAX(mix_used, _throttle_rpy_mix_desired));
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi::rate_controller_run_dt(const Vector3f& gyro, float dt)
{
    // take a copy of the target so that it can't be changed from under us.
    Vector3f ang_vel_body = _ang_vel_body;

    // boost angle_p/pd each cycle on high throttle slew
    update_throttle_gain_boost();

    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    ang_vel_body += _sysid_ang_vel_body;

    _rate_gyro = gyro;
    _rate_gyro_time_us = AP_HAL::micros64();

    _motors.set_roll(get_rate_roll_pid().update_all(ang_vel_body.x, gyro.x,  dt, _motors.limit.roll, _pd_scale.x) + _actuator_sysid.x);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(get_rate_pitch_pid().update_all(ang_vel_body.y, gyro.y,  dt, _motors.limit.pitch, _pd_scale.y) + _actuator_sysid.y);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(get_rate_yaw_pid().update_all(ang_vel_body.z, gyro.z,  dt, _motors.limit.yaw, _pd_scale.z) + _actuator_sysid.z);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);

    _pd_scale_used = _pd_scale;

    control_monitor_update();
}

// reset the rate controller target loop updates
void AC_AttitudeControl_Multi::rate_controller_target_reset()
{
    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();
    _pd_scale = VECTORF_111;
}

// run the rate controller using the configured _dt and latest gyro
void AC_AttitudeControl_Multi::rate_controller_run()
{
    // Set motors to use rate controller
    this->_motors.set_use_LLC(false);
    this->new_file = true;

    Vector3f gyro_latest = _ahrs.get_gyro_latest();
    rate_controller_run_dt(gyro_latest, _dt);
}

// run low level attitude controller
void AC_AttitudeControl_Multi::llc_controller_run()
{   
    // Set motors to use LLC
    this->_motors.set_use_LLC(true);

    if(this->new_flight)
        init_flight_time = AP_HAL::millis() / 1E3;
    
    // float t = AP_HAL::millis() / 1E3 - init_flight_time;
    float x_ref = 0.0f, y_ref = 0.0f, z_ref = 2.0f;
    float x_dot_ref = 0.0f, y_dot_ref = 0.0f, z_dot_ref = 0.0f;
    float x_ddot_ref = 0.0f, y_ddot_ref = 0.0f, z_ddot_ref = 0.0f;
    float x_dddot_ref = 0.0f, y_dddot_ref = 0.0f, z_dddot_ref = 0.0f;

    // Desired path
    Vector3f x_d(x_ref, y_ref, -z_ref);
    Vector3f x_d_dot(x_dot_ref, y_dot_ref, -z_dot_ref);
    Vector3f x_d_ddot(x_ddot_ref, y_ddot_ref, -z_ddot_ref);
    Vector3f x_d_dddot(x_dddot_ref, y_dddot_ref, -z_dddot_ref);
    float psi_d = 0.0f;
    float psi_d_dot = 0.0f;

    // Desired attitude initialization
    Quaternion q_d(1.0f, 0.0f, 0.0f, 0.0f);
    Vector3f omega_d(0.0f, 0.0f, 0.0f);

    // Parameters
    float mass = 0.520;
    float g = 9.81f;
    float T = mass*g;
    Vector3f e_z(0.0f, 0.0f, 1.0f);

    // Drone data initialization
    Vector3f x(0.0f, 0.0f, 0.0f);
    Vector3f x_dot(0.0f, 0.0f, 0.0f);
    Vector3f x_ddot(0.0f, 0.0f, 0.0f);

    // Control gains
    Matrix3f kp1(-1.0f, 0.0f, 0.0f,
                 0.0f, -1.0f, 0.0f,
                 0.0f, 0.0f, -3.0f);

    Matrix3f kd1(-0.5f, 0.0f, 0.0f,
                0.0f, -0.5, 0.0f,
                0.0f, 0.0f, -0.5f);

    
    Vector3f u_d(0.0f, 0.0f, 0.0f);
    Vector3f u_d_dot(0.0f, 0.0f, 0.0f);
    
    // Quaternion q_body, q_d, q_error;
    Quaternion q_body, q_error;
    _ahrs.get_quat_body_to_ned(q_body);
    _rate_gyro = _ahrs.get_gyro_latest();


    omega_d = {0.0f, 0.0f, 0.0f};


    // Normalizing quaternions
    q_d.normalize();
    q_body.normalize();

    if(this->new_flight) {
        last_q_body = q_body;
        this->new_flight = false;
    }

    // Checking sign changes in quaternions
    if(q_body.q1*last_q_body.q1 + q_body.q2*last_q_body.q2 + q_body.q3*last_q_body.q3 + q_body.q4*last_q_body.q4 < 0.0f) {
        q_body.q1 = -q_body.q1;
        q_body.q2 = -q_body.q2;
        q_body.q3 = -q_body.q3;
        q_body.q4 = -q_body.q4;
    }
    last_q_body = q_body;

    if(_ahrs.get_relative_position_NED_home(x) && _ahrs.get_velocity_NED(x_dot)) 
    {   
        x_ddot = _ahrs.get_accel_ef(); // Acceleration in NED inertial frame
        x_ddot = x_ddot + e_z*g;

        // Errors
        Vector3f xe = x - x_d;
        Vector3f xe_dot = x_dot - x_d_dot;
        Vector3f xe_ddot = x_ddot - x_d_ddot;

        // Control law
        u_d = kp1 * xe + kd1 * xe_dot - e_z * mass * g + x_d_ddot * mass;
        u_d_dot = kp1 * xe_dot + kd1 * xe_ddot + x_d_dddot * mass;

        // std::cout << "u_d: " << u_d.x << ", " << u_d.y << ", " << u_d.z << std::endl;

        if(ref_received)
        {
            u_d = u_d_received;
            // std::cout << "Reference received: " << u_d.x << " " << u_d.y << " " << u_d.z << std::endl;
            u_d_dot = u_d_dot_received;
            // Quaternion ud_q(0.0f, u_d.x, u_d.y, u_d.z);
            // Quaternion ud_ned = q_body * ud_q * q_body.inverse() ;
            // u_d = {ud_ned.q2, ud_ned.q3, ud_ned.q4};
            // u_d magnitude
            // std::cout << "Reference received magnitude: " << u_d.length() << std::endl;
            Matrix3f R =  _ahrs.get_rotation_body_to_ned();
            R.normalize();
            // std::cout << "R = " << R.a.x << " " << R.a.y << " " << R.a.z << std::endl;
            // std::cout << R.b.x << " " << R.b.y << " " << R.b.z << std::endl;
            // std::cout << R.c.x << " " << R.c.y << " " << R.c.z << std::endl;
            u_d = R * u_d;
            // std::cout << "Reference received transformed: " << u_d.x << " " << u_d.y << " " << u_d.z << std::endl;
            // std::cout << "Reference received transformed magnitude: " << u_d.length() << std::endl;
            // R.transpose();
            // u_d = R * u_d;
            // std::cout << "Reference received transformed back: " << u_d.x << " " << u_d.y << " " << u_d.z << std::endl;
            
            // std::cout << "==============================================================================" << std::endl;
            // std::cout << "Reference received: " << u_d.x << " " << u_d.y << " " << u_d.z << std::endl;
            // std::cout << "Reference received: " << u_d_dot.x << " " << u_d_dot.y << " " << u_d_dot.z << std::endl;
            // std::cout << "==============================================================================" << std::endl;
        }

        // Desired attitude
        Vector3f u_d_norm = u_d.normalized();
        Vector3f u_d_dot_norm = u_d_dot / u_d.length() - u_d * (u_d * u_d_dot) / powf(u_d.length(), 3.0f);

        Quaternion q_dxy(1.0f/2.0f * sqrtf(-2*u_d_norm.z + 2),
                         u_d_norm.y / sqrtf(-2*u_d_norm.z + 2),
                         -u_d_norm.x / sqrtf(-2*u_d_norm.z + 2),

                         0.0f);

        Quaternion q_dz(cosf(psi_d/2.0f), 
                             0.0f, 
                             0.0f, 
                             sinf(psi_d/2.0f));

        q_d = q_dxy * q_dz;
        
        omega_d = {-sinf(psi_d)*u_d_dot_norm.x + cosf(psi_d)*u_d_dot_norm.y + u_d_dot_norm.z*(sinf(psi_d)*u_d_norm.x - cosf(psi_d)*u_d_norm.y)/(u_d_norm.z - 1.0f),
                             -cosf(psi_d)*u_d_dot_norm.x - sinf(psi_d)*u_d_dot_norm.y + u_d_dot_norm.z*(cosf(psi_d)*u_d_norm.x + sinf(psi_d)*u_d_norm.y)/(u_d_norm.z - 1.0f),
                             psi_d_dot + (u_d_norm.x*u_d_dot_norm.y - u_d_norm.y*u_d_dot_norm.x)/(u_d_norm.z - 1.0f)};

        // Thrust
        T = u_d.length();      
    }
    
    // if(!ref_received)
    // {
    //     // pi/4 yaw
    //     // q_d = Quaternion(0.9239f, 0.0f, 0.0f, 0.3827f);
    //     // -pi/yaw
    //     // q_d = Quaternion(0.7071f, 0.0f, 0.0f, -0.7071f);
    //     // -pi/4 yaw
    //     q_d = Quaternion(0.9239f, 0.0f, 0.0f, -0.3827f);
    // }

    q_d.q1 = 1.0f; q_d.q2 = 0.0f; q_d.q3 = 0.0f; q_d.q4 = 0.0f;
    q_error = q_d.inverse() * q_body;
    _attitude_ang_error = q_error;

    // Rates
    Vector3f omega(_rate_gyro.x, _rate_gyro.y, _rate_gyro.z);

    // Rate error
    Vector3f q_error_v(q_error.q2, q_error.q3, q_error.q4);
    Vector3f omega_error;
    omega_error = omega - omega_d;


    // Gain matrix
    Matrix3f k1(2.0, 0.0f, 0.0f,
            0.0f, 2.0f, 0.0f,
            0.0f, 0.0f, 2.0f);

    // k1 = k1*0.0f;

    Matrix3f k2(0.2f, 0.0f, 0.0f,
                0.0f, 0.2f, 0.0f,
                0.0f, 0.0f, 0.2f);


    k2 = k2*0.0f;
    // Control law for the attitude controller
    Vector3f Tau = -k1 * q_error_v - k2 * omega_error;

    // Control action
    T = mass*g*0.3f;
    float u[4] = {T, Tau[0], Tau[1], Tau[2]};
    // float u[4] = {T, 0.0f, 0.0f, 0.0f};

    // Motor angular velocities computation
    // l -> d: distance from the center of the drone to the propellers
    // k -> b: thrust coefficient
    // b -> k: drag coefficient
    float b = 2.980E-6, d = 0.3181f, k = 1.140E-7*10.0f;
    float omega_motors[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float c1 = 1/(4.0f*b), c2 = sqrtf(2.0f)/(4.0f*b*d), c3 = 1/(4.0f*k);
    omega_motors[0] = c1*u[0] - c2*u[1] + c2*u[2] + c3*u[3];
    omega_motors[1] = c1*u[0] + c2*u[1] - c2*u[2] + c3*u[3];
    omega_motors[2] = c1*u[0] + c2*u[1] + c2*u[2] - c3*u[3];
    omega_motors[3] = c1*u[0] - c2*u[1] - c2*u[2] - c3*u[3];
    
    // Motor angular velocities limits
    for(int i = 0; i < 4; i++){
        // Limit omega_motors
        omega_motors[i] = omega_motors[i] < 0.0f ? 0.0f : sqrtf(omega_motors[i])/838.0f;
        omega_motors[i] = omega_motors[i] > 1.0f ? 1.0f : omega_motors[i];
    }

    // for(int i = 0; i < 4; i++){
    //     // Limit omega_motors
    //     omega_motors[i] = omega_motors[i] < 0.0f ? 0.0f : sqrtf(omega_motors[i])/3220.0f;
    //     omega_motors[i] = omega_motors[i] > 1.0f ? 1.0f : omega_motors[i];
    // }
    

    // Send motor angular velocities to the motors
    this->_motors.set_omega1(omega_motors[0]);
    this->_motors.set_omega2(omega_motors[1]);
    this->_motors.set_omega3(omega_motors[2]);
    this->_motors.set_omega4(omega_motors[3]);

    // this->_motors.set_omega1(0.0f);
    // this->_motors.set_omega2(0.0f);
    // this->_motors.set_omega3(0.5f);
    // this->_motors.set_omega4(0.0f);

    // Print info
    // std::cout << "T: " << T << std::endl;
    // std::cout << "Tau: " << Tau[0] << " " << Tau[1] << " " << Tau[2] << std::endl;
    // std::cout << "u: " << u[0] << " " << u[1] << " " << u[2] << " " << u[3] << std::endl;
    // std::cout << "omega_motors: " << omega_motors[0] << " " << omega_motors[1] << " " << omega_motors[2] << " " << omega_motors[3] << std::endl;
    
    // To create a new file with time stamp
    // if(this->new_file) {
    //     // Time stamp
    //     this->new_file = false;  
    //     auto td = std::time(nullptr);
    //     auto tm = *std::localtime(&td);
    //     char timestamp[20];
    //     std::strftime(timestamp, sizeof(timestamp), "%m-%d_%H-%M-%S", &tm);

    //     this->att_filename = "/home/olara/Desktop/plots_ap/attitude_data/attitude_data_" + std::string(timestamp) + ".txt";
    //     this->pos_filename = "/home/olara/Desktop/plots_ap/position_data/position_data_" + std::string(timestamp) + ".txt";
    // }

    // // Open file to save q_d, q_body, q_error along with time
    // std::ofstream attitude_data(this->att_filename, std::ios_base::app);

    // if (!attitude_data.is_open()) {
    //     std::cerr << "Error opening file" << std::endl;
    // } else {
    //     // Write time, q_d, q_body, q_error to file
    //     attitude_data << t << " "; // Time in seconds
    //     attitude_data << q_d.q1 << " " << q_d.q2 << " " << q_d.q3 << " " << q_d.q4 << " "; // q_d quaternion
    //     attitude_data << q_body.q1 << " " << q_body.q2 << " " << q_body.q3 << " " << q_body.q4 << " "; // q_body quaternion
    //     attitude_data << q_error.q1 << " " << q_error.q2 << " " << q_error.q3 << " " << q_error.q4 << " "; // q_error quaternion
    //     attitude_data << omega_d.x << " " << omega_d.y << " " << omega_d.z << " "; // omega_d vector
    //     attitude_data << omega.x << " " << omega.y << " " << omega.z << " "; // omega vector
    //     attitude_data << u[0] << " " << u[1] << " " << u[2] << " " << u[3] << " "; // Control action
    //     attitude_data << omega_motors[0] << " " << omega_motors[1] << " " << omega_motors[2] << " " << omega_motors[3] << " "; // Motor angular velocities
    //     attitude_data << u_d.x << " " << u_d.y << " " << u_d.z << " "; // u_d vector
    //     attitude_data << u_d_dot.x << " " << u_d_dot.y << " " << u_d_dot.z << std::endl; // u_d_dot vector
    // }

    // attitude_data.close();

    // // Open file to save position
    // std::ofstream position_data(this->pos_filename, std::ios_base::app);

    // // Save x and x_d to file
    // if (!position_data.is_open()) {
    //     std::cerr << "Error opening file" << std::endl;
    // } else {
    //     // Write time, x, x_d to file
    //     position_data << t << " "; // Time in seconds
    //     position_data << x[0] << " " << x[1] << " " << x[2] << " "; // x vector
    //     position_data << x_dot[0] << " " << x_dot[1] << " " << x_dot[2] << " "; // x_dot vector
    //     position_data << x_ddot[0] << " " << x_ddot[1] << " " << x_ddot[2] << " "; // x_ddot vector
    //     position_data << x_d[0] << " " << x_d[1] << " " << x_d[2] << " "; // x_d vector
    //     position_data << x_d_dot[0] << " " << x_d_dot[1] << " " << x_d_dot[2] << " "; // x_d_dot vector
    //     position_data << x_d_ddot[0] << " " << x_d_ddot[1] << " " << x_d_ddot[2] << " "; // x_d_ddot vector
    //     position_data << x_d_dddot[0] << " " << x_d_dddot[1] << " " << x_d_dddot[2] << std::endl; // x_d_dddot vector
    // }

    // position_data.close();
}   

// Recived the desired position and orientation
void AC_AttitudeControl_Multi::llc_set_virtual_ctrl(const Vector3f& u_d, const Vector3f& u_d_dot)
{
    // Set the desired position and orientation
    this->u_d_received = u_d;
    this->u_d_dot_received = u_d_dot;
    this->ref_received = true;
    // float delta_t = AP_HAL::millis() / 1E3 - last_time;
    last_time = AP_HAL::millis() / 1E3;
//     std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
//     std::cout << "Reference received" << std::endl;
//     std::cout << "Time: " << delta_t << std::endl;
//     std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
}

// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}

void AC_AttitudeControl_Multi::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    _pid_rate_roll.set_notch_sample_rate(sample_rate);
    _pid_rate_pitch.set_notch_sample_rate(sample_rate);
    _pid_rate_yaw.set_notch_sample_rate(sample_rate);
#endif
}
