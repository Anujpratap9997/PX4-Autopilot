/**
 * @file CapableControl.cpp
 * @see CapableControl.hpp
 */

#include "CapableControl.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>

#include <cstdarg>

#ifdef __PX4_NUTTX
#include <poll.h>
#include <unistd.h>
#endif

#include <cstdlib>
#include <cstring>

using namespace capable;

ModuleBase::Descriptor CapableControl::desc{task_spawn, custom_command, print_usage};

namespace
{
// PX4 NED/FRD <-> controller NWU/FLU: a 180 deg rotation about x on both sides.
inline V3 to_zup(float x, float y, float z) { return V3(x, -y, -z); }
inline V3 to_frd(const V3 &v) { return V3(v(0), -v(1), -v(2)); }
inline Quat quat_to_zup(const float q[4]) { return Quat{q[0], q[1], -q[2], -q[3]}; }

const char *scenario_name(int type)
{
	static const char *const names[] = {"hover", "translate", "circle", "pursuit", "dive", "sysid", "tilt", "accel"};
	return type >= 0 && type < (int)(sizeof(names) / sizeof(names[0])) ? names[type] : "test";
}
}

// Abort of a test: console and log (CAPABLE_ABORTED, parsed by the tools), QGC, and the end of fly's wait.
#define CC_ABORT(...) end_test(false, __VA_ARGS__)

void CapableControl::end_test(bool ok, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(_end_msg, sizeof(_end_msg), fmt, ap);
	va_end(ap);

	if (!ok) { PX4_ERR("CAPABLE_ABORTED: %s", _end_msg); }

	mavlink_vasprintf(ok ? _MSG_PRIO_INFO : _MSG_PRIO_CRITICAL, &_mavlink_log_pub, "Capable Control %s: %s",
			  ok ? "done" : "aborted", _end_msg);
	_end_ok = ok;
	_end_seq.fetch_add(1);
}

void CapableControl::publish_setpoints(const Quat &qd, const V3 &w_des, double thrust, const capable::Ref *ref)
{
	// controller frames NWU / FLU -> PX4 NED / FRD (180 deg about x)
	const hrt_abstime t = hrt_absolute_time();
	const float thr = _f_max > 0.f ? (float)(thrust / (4.0 * (double)_f_max)) : 0.f;
	vehicle_attitude_setpoint_s a{};
	a.timestamp = t;
	a.q_d[0] = (float)qd.w; a.q_d[1] = (float)qd.x; a.q_d[2] = (float) - qd.y; a.q_d[3] = (float) - qd.z;
	a.thrust_body[2] = -thr;
	_att_sp_pub.publish(a);
	vehicle_rates_setpoint_s r{};
	r.timestamp = t;
	r.roll = (float)w_des(0); r.pitch = (float) - w_des(1); r.yaw = (float) - w_des(2);
	r.thrust_body[2] = -thr;
	_rates_sp_pub.publish(r);

	if (ref) {
		vehicle_local_position_setpoint_s s{};
		s.timestamp = t;
		s.x = (float)ref->p(0); s.y = (float) - ref->p(1); s.z = (float) - ref->p(2);
		s.vx = (float)ref->v(0); s.vy = (float) - ref->v(1); s.vz = (float) - ref->v(2);
		s.acceleration[0] = (float)ref->a(0); s.acceleration[1] = (float) - ref->a(1); s.acceleration[2] = (float) - ref->a(2);
		s.thrust[0] = s.thrust[1] = s.thrust[2] = NAN;
		s.yaw = (float)atan2(-sin(ref->yaw), cos(ref->yaw));
		s.yawspeed = NAN;
		_lpos_sp_pub.publish(s);
	}
}

CapableControl::CapableControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

CapableControl::~CapableControl()
{
	perf_free(_loop_perf);
	perf_free(_interval_perf);
}

// Every module parameter. PX4 logs only the parameters in use when a log opens; on hardware the log
// can open before the first fly, so they are looked up at start and printed (logged) at every fly.
static constexpr const char *CC_PARAMS[] = {
	"CC_MASS", "CC_IXX", "CC_IYY", "CC_IZZ", "CC_ACC_MAX_G", "CC_TW_MAX", "CC_V_MAX", "CC_ATT_WN", "CC_ATT_ZETA",
	"CC_YAW_WN", "CC_YAW_ZETA", "CC_DRAG", "CC_POS_KP", "CC_VEL_KD", "CC_ROT_FMAX", "CC_THR_MODEL", "CC_TRUTH_SRC",
	"CC_UPRIGHT", "CC_ATT_MODE", "CC_MOT_TAU", "CC_COMP_TAU", "CC_RIG_EN", "CC_RIG_THR", "CC_RIG_TILT",
	"CC_RIG_RATE", "CC_RIG_RAMP", "CC_RIG_YAW", "CC_ENG_RAMP", "CC_DOB_EN", "CC_DOB_TAU", "CC_DOB_MTAU",
	"CC_DOB_MAXG", "CC_DOB_TFRAC"
};

/* CAPABLE_PARAMS lines: name=value (without CC_), float32 values exact (%.9g); rig_log.py reads them. */
void CapableControl::print_params() const
{
	char line[112];
	size_t n = 0;

	for (const char *name : CC_PARAMS) {
		const param_t h = param_find(name);

		if (h == PARAM_INVALID) { continue; }

		char item[40];

		if (param_type(h) == PARAM_TYPE_INT32) {
			int32_t v = 0;
			param_get(h, &v);
			snprintf(item, sizeof(item), " %s=%d", name + 3, (int)v);

		} else {
			float v = 0.f;
			param_get(h, &v);
			snprintf(item, sizeof(item), " %s=%.9g", name + 3, (double)v);
		}

		if (n > 0 && n + strlen(item) > 88) { PX4_INFO("CAPABLE_PARAMS:%s", line); n = 0; }

		n += snprintf(line + n, sizeof(line) - n, "%s", item);
	}

	if (n > 0) { PX4_INFO("CAPABLE_PARAMS:%s", line); }
}

bool CapableControl::init()
{
	for (const char *name : CC_PARAMS) { (void)param_find(name); }   // in use from now on: logged


	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

// ---------------------------------------------------------------- external mode plumbing

void CapableControl::register_mode()
{
	register_ext_component_request_s req{};
	req.timestamp = hrt_absolute_time();
	strncpy(req.name, "Capable Control", sizeof(req.name) - 1);
	req.request_id = REQUEST_ID;
	req.px4_ros2_api_version = 1;
	req.register_arming_check = true;
	req.register_mode = true;
	_register_request_pub.publish(req);
}

void CapableControl::check_registration()
{
	register_ext_component_reply_s reply;
	int tries = register_ext_component_reply_s::ORB_QUEUE_LENGTH;

	while (_register_reply_sub.update(&reply) && --tries >= 0) {
		if (reply.request_id == REQUEST_ID && reply.success) {
			_arming_check_id = reply.arming_check_id;
			_mode_id = reply.mode_id;
			PX4_INFO("mode 'Capable Control' registered: nav_state %d, arming check %d", _mode_id, _arming_check_id);
			configure_mode();
			break;
		}
	}
}

void CapableControl::configure_mode()
{
	// Direct actuators: every control flag off, no PX4 position/velocity/attitude/rate controller and no control
	// allocator runs in this mode. This module writes actuator_motors itself.
	setpoint_config_s cfg{};
	cfg.timestamp = hrt_absolute_time();
	cfg.source_id = _mode_id;
	cfg.type = setpoint_config_s::TYPE_DIRECT_ACTUATORS;
	cfg.should_apply = true;
	_setpoint_config_pub.publish(cfg);
	_last_config_publish = cfg.timestamp;
}

void CapableControl::reply_arming_check(int8_t request_id)
{
	arming_check_reply_s r{};
	r.timestamp = hrt_absolute_time();
	r.request_id = request_id;
	r.registration_id = _arming_check_id;
	r.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	r.num_events = 0;
	r.can_arm_and_run = true;
	r.mode_req_angular_velocity = true;
	r.mode_req_attitude = true;
	// The rig (CC_RIG_EN 1) has no position source: -F and -V do not use the real position.
	// Read live, so the requirement is right before the first fly loads the parameters.
	int32_t rig = 0;
	param_get(param_find("CC_RIG_EN"), &rig);
	r.mode_req_local_alt = rig != 1;
	r.mode_req_local_position = rig != 1;
	_arming_check_reply_pub.publish(r);
}

void CapableControl::send_command(uint32_t command, float p1, float p2)
{
	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = command;
	cmd.param1 = p1;
	cmd.param2 = p2;
	cmd.target_system = _vehicle_status.system_id;
	cmd.target_component = _vehicle_status.component_id;
	cmd.source_system = _vehicle_status.system_id;
	cmd.source_component = _vehicle_status.component_id;
	cmd.from_external = false;
	_vehicle_command_pub.publish(cmd);
}

// ---------------------------------------------------------------- mixer (SIH quad X)

static float get_param_f(const char *name, bool &ok)
{
	float v = NAN;
	const param_t h = param_find(name);

	if (h == PARAM_INVALID || param_get(h, &v) != PX4_OK) { PX4_ERR("param %s missing", name); ok = false; }

	return v;
}

bool CapableControl::load_vehicle()
{
	bool ok = true;
	// Reloading the parameters rebuilds the controller below, which would also throw away the disturbance
	// the observer has learnt. A fly continuing from a hover keeps it, so the test does not start by
	// re-learning the same mass error; a fresh engagement or a flight from the ground calls reset_dob().
	V3 keep_d_perp{};
	double keep_d_thrust = 0.0;
	const bool keep_dob = _dob_en && _state == capable_control_status_s::STATE_HOLD;

	if (keep_dob) { _controller.get_dob_estimate(keep_d_perp, keep_d_thrust); }

	// --- vehicle model and gains (defaults = Table II chaser)
	Capability c = capable::chaser();
	c.mass = get_param_f("CC_MASS", ok);
	c.inertia = V3(get_param_f("CC_IXX", ok), get_param_f("CC_IYY", ok), get_param_f("CC_IZZ", ok));
	c.max_lateral_accel_g = get_param_f("CC_ACC_MAX_G", ok);
	c.max_thrust_to_weight = get_param_f("CC_TW_MAX", ok);
	c.max_speed = get_param_f("CC_V_MAX", ok);
	c.omega_n_att = get_param_f("CC_ATT_WN", ok);
	c.zeta_att = get_param_f("CC_ATT_ZETA", ok);
	c.omega_n_yaw = get_param_f("CC_YAW_WN", ok);
	c.zeta_yaw = get_param_f("CC_YAW_ZETA", ok);
	c.linear_drag = get_param_f("CC_DRAG", ok);
	const double kp = get_param_f("CC_POS_KP", ok), kd = get_param_f("CC_VEL_KD", ok);
	c.kp_pos = V3(kp, kp, kp);
	c.kd_vel = V3(kd, kd, kd);
	int32_t upright = 0;
	param_get(param_find("CC_UPRIGHT"), &upright);
	c.upright = upright == 1;
	_upright = c.upright;
	_controller = capable::CapableFlightController(c);
	param_get(param_find("CC_ATT_MODE"), &_att_mode);
	_mot_tau = get_param_f("CC_MOT_TAU", ok);
	_comp_tau = get_param_f("CC_COMP_TAU", ok);
	_J = c.inertia;
	_mass = c.mass;
	_kdrag = c.linear_drag;
	int32_t rig = 0;
	param_get(param_find("CC_RIG_EN"), &rig);
	_rig = rig == 1;
	int32_t rig_yaw = 1;
	param_get(param_find("CC_RIG_YAW"), &rig_yaw);
	_rig_yaw = rig_yaw == 1;
	_rig_thr = get_param_f("CC_RIG_THR", ok);
	_rig_tilt = get_param_f("CC_RIG_TILT", ok);
	_rig_rate = get_param_f("CC_RIG_RATE", ok);
	_rig_ramp = get_param_f("CC_RIG_RAMP", ok);
	_eng_ramp = get_param_f("CC_ENG_RAMP", ok);

	// --- (6) acceleration disturbance observer. The rig and the fake-translation tests have no physical
	// translation to measure, so it is never enabled there whatever CC_DOB_EN says.
	int32_t dob = 0;
	param_get(param_find("CC_DOB_EN"), &dob);
	_dob_en = dob == 1 && !_rig;
	_controller.configure_dob(_dob_en, get_param_f("CC_DOB_TAU", ok), get_param_f("CC_DOB_MTAU", ok),
				  get_param_f("CC_DOB_MAXG", ok), get_param_f("CC_DOB_TFRAC", ok), _mot_tau);

	if (keep_dob && _dob_en) { _controller.set_dob_estimate(keep_d_perp, keep_d_thrust); }

	// --- mixer: wrench [T; Mx; My; Mz] (FRD) = A f, column i = F_max [1, -PY_i, PX_i, KM_i]
	// (same convention as ActuatorEffectivenessRotors with the thrust axis -z)
	_f_max = get_param_f("CC_ROT_FMAX", ok);
	param_get(param_find("CC_TRUTH_SRC"), &_truth_src);
	param_get(param_find("CC_THR_MODEL"), &_thr_model);
	int32_t count = 0;
	param_get(param_find("CA_ROTOR_COUNT"), &count);

	if (count != 4) { PX4_ERR("CA_ROTOR_COUNT = %d, need 4", (int)count); ok = false; }

	char name[20];

	for (int i = 0; i < 4 && ok; i++) {
		snprintf(name, sizeof(name), "CA_ROTOR%d_PX", i); _rotor_px[i] = get_param_f(name, ok);
		snprintf(name, sizeof(name), "CA_ROTOR%d_PY", i); _rotor_py[i] = get_param_f(name, ok);
		snprintf(name, sizeof(name), "CA_ROTOR%d_KM", i); _rotor_km[i] = get_param_f(name, ok);
		_A(0, i) = _f_max;
		_A(1, i) = -_rotor_py[i] * _f_max;
		_A(2, i) = _rotor_px[i] * _f_max;
		_A(3, i) = _rotor_km[i] * _f_max;
	}

	_mixer_ok = ok && matrix::inv(_A, _A_inv);

	if (_mixer_ok) {
		PX4_INFO("CAPABLE_VEHICLE: mass %.3f I (%.4f %.4f %.4f) Fmax %.3f model %s r0 (%.4f %.4f km %.4f)",
			 c.mass, c.inertia(0), c.inertia(1), c.inertia(2), (double)_f_max, _thr_model ? "quadratic" : "linear",
			 (double)_rotor_px[0], (double)_rotor_py[0], (double)_rotor_km[0]);
	}

	print_params();

	return _mixer_ok;
}

/* Attitude torque compensation for motor lag (CC_ATT_MODE). Input: the paper's (5.7b) torque, FLU.
 * 1: lead (T_m s + 1)/(T_c s + 1) on the torque: cancels the motor pole, leaves a T_c one.
 * 2: incremental (INDI) form: tau = tau_hat + J (alpha_des - wdot_hat) with alpha_des = tau_(5.7b) / J,
 *    tau_hat = motor model (T_m) of the achieved torque, then filtered by T_c like the measured wdot. */
/* Stage B: one step of the virtual point mass. Thrust along the REAL body z axis, the commanded
 * collective delayed by the motor model (CC_MOT_TAU), gravity and the (5.2) linear drag; the ground
 * (the take-off height) holds it until there is enough thrust to lift off. */
void CapableControl::virtual_step(const Quat &q, double thrust, double dt)
{
	const double a = exp(-dt / fmax(_mot_tau, 1e-3));
	_v_thrust = a * _v_thrust + (1.0 - a) * thrust;
	const M3 R = to_rotation_matrix(q);
	const V3 b3(R(0, 2), R(1, 2), R(2, 2));
	const V3 acc = b3 * (_v_thrust / _mass) - V3(0.0, 0.0, GRAVITY) - _vv * _kdrag;
	_vv += acc * dt;
	_vp += _vv * dt;

	if (_vp(2) < _v_ground) {
		_vp(2) = _v_ground;

		if (_vv(2) < 0.0) { _vv(2) = 0.0; }
	}
}

/* Watchdog: the attitude the controller uses must be fresh (< 100 ms old). */
const char *CapableControl::state_stale(hrt_abstime now) const
{
	const hrt_abstime ts = (_use_ground_truth && _truth_src == 1) ? _visual_odometry.timestamp : att().timestamp;
	return (ts == 0 || now > ts + 100_ms) ? "state older than 100 ms" : nullptr;
}

/* Rig safety (CC_RIG_EN): returns the reason when the state is outside the rig limits, else nullptr. */
const char *CapableControl::rig_violation(const Quat &q, const V3 &w) const
{
	if (!_rig) { return nullptr; }

	const double tilt = acos(fmin(fmax(to_rotation_matrix(q)(2, 2), -1.0), 1.0));

	if (tilt > _rig_tilt * M_PI / 180.0) { return "tilt above CC_RIG_TILT"; }

	if (w.abs().max() > _rig_rate * M_PI / 180.0) { return "body rate above CC_RIG_RATE"; }

	return nullptr;
}

/* Output limits. Free flight: after an RC switch-in, the collective blended from the stock one over CC_ENG_RAMP
 * (torque at once). Rig: no yaw torque when the yaw axis is locked (CC_RIG_YAW 0), soft-start scale on thrust and
 * torque, then the collective cap. */
void CapableControl::limit_output(double &thrust, V3 &torque, double now_s, float &scale, bool &capped) const
{
	scale = 1.f;
	capped = false;

	if (!_rig) {
		if (_engage_thrust >= 0.0 && _eng_ramp > 0.0) {
			scale = (float)fmin(1.0, fmax(0.0, now_s / _eng_ramp));
			thrust = _engage_thrust + (thrust - _engage_thrust) * (double)scale;
		}

		return;
	}

	if (!_rig_yaw) { torque(2) = 0.0; }   // yaw locked on the rig: the motor range goes to roll and pitch

	if (_rig_ramp > 0.0) { scale = (float)fmin(1.0, fmax(0.0, now_s / _rig_ramp)); }

	const double cap = _rig_thr * 4.0 * (double)_f_max;

	if (_engage_thrust >= 0.0) {
		// switched in from Stabilized: full torque at once, collective blended from the stock one to the cap
		if (thrust > cap) { thrust = cap; capped = true; }

		thrust = _engage_thrust + (thrust - _engage_thrust) * (double)scale;
		return;
	}

	thrust *= (double)scale;
	torque = torque * (double)scale;

	if (thrust > cap) { thrust = cap; capped = true; }
}

/* Sysid excitation (PHASE_SYSID): a +-A square wave whose sign changes pseudo-randomly every 24 ms
 * (bandwidth to about 20 Hz), on roll for 4 s, then pitch, then yaw. Amplitude about 5 % of the roll/pitch
 * torque authority: the vehicle stays within a few degrees of level while the closed loop fights it. */
V3 CapableControl::sysid_injection(double t)
{
	const int axis = (int)(t / 4.0);

	if (t < 0.0 || axis > 2) { return V3(); }

	uint32_t n = (uint32_t)(t / 0.024) + 977u * (uint32_t)axis;
	n = (n ^ 61u) ^ (n >> 16); n *= 9u; n ^= n >> 4; n *= 0x27d4eb2du; n ^= n >> 15;   // integer hash
	const double sgn = (n & 1u) ? 1.0 : -1.0;
	const double amp = (axis == 2 ? 0.15 : 0.5) * _sysid_scale;
	V3 v{};
	v(axis) = sgn * amp;
	return v;
}

V3 CapableControl::compensate(const V3 &tau_paper, const V3 &w, double dt)
{
	if (_att_mode == 0) { return tau_paper; }

	if (!_comp_init) {
		_lead_x = tau_paper; _m_hat = tau_paper; _tau_hat = tau_paper; _tau_ach_prev = tau_paper;
		_wdot_hat.zero(); _w_prev = w; _comp_init = true;
		return tau_paper;
	}

	if (dt <= 0.0) { return tau_paper; }

	const double a = exp(-dt / _mot_tau), b = exp(-dt / _comp_tau);

	if (_att_mode == 1) {
		_lead_x = _lead_x * b + tau_paper * (1.0 - b);
		const double r = _mot_tau / _comp_tau;
		return tau_paper * r + _lead_x * (1.0 - r);
	}

	const V3 wdot = (w - _w_prev) / dt;
	_w_prev = w;
	_wdot_hat = _wdot_hat * b + wdot * (1.0 - b);
	_m_hat = _m_hat * a + _tau_ach_prev * (1.0 - a);
	_tau_hat = _tau_hat * b + _m_hat * (1.0 - b);
	const V3 tau_indi = _tau_hat + tau_paper - V3(_J(0) * _wdot_hat(0), _J(1) * _wdot_hat(1), _J(2) * _wdot_hat(2));

	if (_att_mode == 3) {   // INDI followed by the same motor-lag lead as mode 1
		_lead_x = _lead_x * b + tau_indi * (1.0 - b);
		const double r = _mot_tau / _comp_tau;
		return tau_indi * r + _lead_x * (1.0 - r);
	}

	return tau_indi;
}

void CapableControl::read_state(const vehicle_angular_velocity_s &rates, V3 &p, V3 &v, Quat &q, V3 &w) const
{
	if (_use_ground_truth && _truth_src == 1) {
		// Gazebo OdometryPublisher via gz_bridge: NED position, q FRD->NED, BODY_FRD velocity and rates.
		const vehicle_odometry_s &o = _visual_odometry;
		const Quat q_ned{o.q[0], o.q[1], o.q[2], o.q[3]};
		const V3 v_ned = to_rotation_matrix(q_ned) * V3(o.velocity[0], o.velocity[1], o.velocity[2]);
		p = to_zup(o.position[0], o.position[1], o.position[2]);
		v = V3(v_ned(0), -v_ned(1), -v_ned(2));
		q = quat_to_zup(o.q);
		w = to_zup(o.angular_velocity[0], o.angular_velocity[1], o.angular_velocity[2]);
		return;
	}

	p = to_zup(lpos().x, lpos().y, lpos().z);
	v = to_zup(lpos().vx, lpos().vy, lpos().vz);

	if (_rig) { p.zero(); v.zero(); }   // the rig holds the vehicle at the pivot; no position estimate there

	q = quat_to_zup(att().q);
	const float *wr = _use_ground_truth ? _gt_angular_velocity.xyz : rates.xyz;
	w = to_zup(wr[0], wr[1], wr[2]);
}

/* Measurement for the (6) observer. vehicle_acceleration is the specific force in body FRD, already
 * corrected for the calibration AND for EKF2's estimated accelerometer bias, and low-passed at
 * IMU_ACCEL_CUTOFF (sensors/vehicle_acceleration). Rotating it with the attitude the controller is flying
 * on and removing gravity gives the kinematic acceleration that a_model predicts. At rest the FRD sample
 * is (0, 0, -g), which maps to (0, 0, +g) in FLU and then to zero here.
 *
 * There is no groundtruth acceleration topic, so -t gets perfect state but the real IMU. */
bool CapableControl::read_accel(hrt_abstime now, const Quat &q, V3 &a) const
{
	if (_acceleration.timestamp == 0 || now > _acceleration.timestamp + 100_ms) { return false; }

	const V3 f_flu = to_zup(_acceleration.xyz[0], _acceleration.xyz[1], _acceleration.xyz[2]);

	if (!PX4_ISFINITE(f_flu(0)) || !PX4_ISFINITE(f_flu(1)) || !PX4_ISFINITE(f_flu(2))) { return false; }

	a = to_rotation_matrix(q) * f_flu - V3(0.0, 0.0, GRAVITY);
	return true;
}

/* Where the measured acceleration is not free-flight model error, the estimate is held rather than
 * learned: the rig's pivot and the ground reaction are not something thrust can fix, the -F / -V tests
 * fly a reference or a virtual vehicle rather than the real one, and without a valid position estimate
 * the EKF's accelerometer bias (which vehicle_acceleration has already subtracted) is not trustworthy. */
bool CapableControl::dob_frozen(uint8_t phase) const
{
	using S = capable_control_status_s;

	if (_rig || _requested.ref_only || _requested.virt) { return true; }

	if (!_use_ground_truth && !(lpos().xy_valid && lpos().z_valid)) { return true; }

	// PHASE_TAKEOFF is a real lift-off only from the ground; started from a hover it is an ordinary
	// airborne transit, where the observer should go on learning.
	return phase == S::PHASE_GROUND_SPINUP || phase == S::PHASE_LAND || phase == S::PHASE_TOUCHDOWN
	       || (_from_ground && phase == S::PHASE_TAKEOFF);
}

void CapableControl::mix(double thrust, const V3 &torque_frd, float u[4], float u_raw[4], bool &saturated) const
{
	// u_raw = unclipped per-rotor thrust fraction f; u = motor command after the thrust curve.
	matrix::Vector<double, 4> w;
	w(0) = thrust; w(1) = torque_frd(0); w(2) = torque_frd(1); w(3) = torque_frd(2);
	const matrix::Vector<double, 4> sol = _A_inv * w;
	saturated = false;

	for (int i = 0; i < 4; i++) {
		u_raw[i] = (float)sol(i);
		saturated |= sol(i) < -1e-9 || sol(i) > 1.0 + 1e-9;
		const double f = fmin(fmax(sol(i), 0.0), 1.0);
		u[i] = (float)(_thr_model == 1 ? sqrt(f) : f);
	}
}

void CapableControl::wrench(const float u[4], double &thrust, V3 &torque_frd) const
{
	matrix::Vector<double, 4> uu;

	for (int i = 0; i < 4; i++) { const double ui = u[i]; uu(i) = _thr_model == 1 ? ui * ui : ui; }

	const matrix::Vector<double, 4> w = _A * uu;
	thrust = w(0);
	torque_frd = V3(w(1), w(2), w(3));
}

void CapableControl::publish_motors(const float u[4])
{
	actuator_motors_s m{};
	m.timestamp = hrt_absolute_time();
	m.timestamp_sample = _last_sample;
	m.reversible_flags = 0;

	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) { m.control[i] = i < 4 ? u[i] : NAN; }

	_actuator_motors_pub.publish(m);

	for (int i = 0; i < 4; i++) { _last_u[i] = u[i]; }
}

/* Free flight: hold here and ask the commander for Position mode (repeated from HOLD every 0.5 s until it
 * switches). When it refuses (no position or attitude), its own failsafe for this mode's requirements takes over. */
void CapableControl::hand_back(const V3 &p, const Quat &q)
{
	const M3 R = to_rotation_matrix(q);
	_hold_p = p;
	_hold_yaw = atan2(R(1, 0), R(0, 0));
	_requested.ref_only = false;   // hold position, on the real state
	_requested.virt = false;
	_last_sample = 0;
	_last_command = 0;
	_handing_back = true;
	_state = capable_control_status_s::STATE_HOLD;
}

/* One capable_control_status sample (FLY and HOLD ticks); also feeds the achieved torque to compensate(). */
void CapableControl::publish_status(hrt_abstime now, double t_fly, double dt, bool pursuit, const V3 &p, const V3 &v,
				    const Quat &q, const V3 &w, const Ref &ref, const V3 &a_raw, const ControlOutput &out,
				    const V3 &tau_cmd, const V3 &tau_inj, const float u[4], const float u_raw[4], bool sat_mixer,
				    float out_scale, bool thr_capped, const V3 &a_meas, bool dob_held, uint32_t compute_us)
{
	using S = capable_control_status_s;
	S s{};
	s.timestamp_sample = now;
	s.state = _state;
	s.state_source = _use_ground_truth ? S::SOURCE_GROUND_TRUTH : S::SOURCE_EKF;
	s.pursuit = pursuit;
	s.upright = _upright;
	s.phase = ref.phase;
	s.t_fly = (float)t_fly;
	s.dt = (float)(_rate_ff ? dt : 0.0);

	for (int i = 0; i < 3; i++) {
		s.pos[i] = p(i); s.vel[i] = v(i); s.rates[i] = w(i);
		s.pos_ref[i] = ref.p(i); s.vel_ref[i] = ref.v(i); s.acc_ref[i] = ref.a(i);
		s.acc_cmd_raw[i] = a_raw(i); s.acc_cmd[i] = out.commanded_acceleration(i);
		s.acc_des[i] = out.desired_acceleration(i); s.e_r[i] = out.e_r(i);
		s.omega_ff[i] = _controller.omega_des_filtered()(i); s.omega_des[i] = out.omega_desired(i);
		s.torque[i] = out.body_torque(i);
	}

	s.q[0] = q.w; s.q[1] = q.x; s.q[2] = q.y; s.q[3] = q.z;
	const Quat &qd = out.desired_quaternion;
	s.q_des[0] = qd.w; s.q_des[1] = qd.x; s.q_des[2] = qd.y; s.q_des[3] = qd.z;
	s.yaw_ref = ref.yaw;
	const Quat qe = multiply(conjugate(q), qd);
	s.att_err = (float)(2.0 * acos(fmin(1.0, fabs(qe.w))));
	s.tilt = (float)acos(fmin(fmax(to_rotation_matrix(q)(2, 2), -1.0), 1.0));
	s.tilt_cmd = (float)out.tilt_rad;
	s.thrust = out.collective_thrust;
	V3 torque_ach_frd;
	wrench(u, s.thrust_ach, torque_ach_frd);
	const V3 torque_ach = to_frd(torque_ach_frd);   // same flip maps back to FLU

	compensate_post(torque_ach);

	for (int i = 0; i < 3; i++) { s.torque_ach[i] = torque_ach(i); s.torque_cmd[i] = tau_cmd(i); s.torque_inj[i] = tau_inj(i); }

	s.att_mode = (uint8_t)_att_mode;
	s.ref_only = _requested.ref_only;
	s.virtual_translation = _requested.virt;
	s.output_scale = out_scale;
	s.thrust_capped = thr_capped;

	for (int i = 0; i < 4; i++) { s.u[i] = u[i]; s.u_raw[i] = u_raw[i]; }

	s.sat_accel = out.saturated_accel;
	s.sat_thrust = out.saturated_thrust;
	s.sat_mixer = sat_mixer;
	s.sat_mixer_paper = out.saturated_mixer;
	s.dob = out.dob_active;
	s.dob_frozen = dob_held;
	s.dob_d_thrust = out.accel_disturbance_thrust;

	for (int i = 0; i < 3; i++) {
		s.acc_meas[i] = a_meas(i);
		s.dob_model[i] = out.accel_model(i);
		s.dob_d[i] = out.accel_disturbance(i);
	}

	s.compute_us = compute_us;
	s.timestamp = hrt_absolute_time();
	_status_pub.publish(s);
}

// ---------------------------------------------------------------- main loop

void CapableControl::Run()
{
	if (should_exit()) {
		_angular_velocity_sub.unregisterCallback();

		if (_mode_id >= 0) {
			unregister_ext_component_s u{};
			u.timestamp = hrt_absolute_time();
			strncpy(u.name, "Capable Control", sizeof(u.name) - 1);
			u.arming_check_id = _arming_check_id;
			u.mode_id = _mode_id;
			_unregister_pub.publish(u);
		}

		exit_and_cleanup(desc);
		return;
	}

	if (!_sent_registration) {
		register_mode();
		_sent_registration = true;
		return;
	}

	if (_mode_id < 0) {
		check_registration();
		return;
	}

	arming_check_request_s req;

	if (_arming_check_request_sub.update(&req)) {
		reply_arming_check(req.request_id);
	}

	_vehicle_status_sub.update(&_vehicle_status);

	// The commander resets a mode's setpoint type to its default (trajectory: PX4's controllers and allocator on)
	// whenever the mode is deactivated. Set direct actuators again ahead of the next activation, and on it.
	const bool config_in_mode = _vehicle_status.nav_state == _mode_id;

	if ((config_in_mode && !_config_in_mode) || (!config_in_mode && hrt_elapsed_time(&_last_config_publish) > 500_ms)) {
		configure_mode();
	}

	_config_in_mode = config_in_mode;
	_acceleration_sub.update(&_acceleration);
	_attitude_sub.update(&_attitude);
	_local_position_sub.update(&_local_position);
	_gt_attitude_sub.update(&_gt_attitude);
	_gt_local_position_sub.update(&_gt_local_position);
	_gt_angular_velocity_sub.update(&_gt_angular_velocity);
	_visual_odometry_sub.update(&_visual_odometry);

	vehicle_angular_velocity_s rates;

	if (_angular_velocity_sub.update(&rates)) {
		perf_count(_interval_perf);
		perf_begin(_loop_perf);
		control_tick(rates);
		perf_end(_loop_perf);
	}
}

void CapableControl::control_tick(const vehicle_angular_velocity_s &rates)
{
	using S = capable_control_status_s;
	const hrt_abstime now = rates.timestamp_sample;
	const bool in_mode = _vehicle_status.nav_state == _mode_id;
	const bool armed = _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	const float zero[4] {};
	const bool switched_in = in_mode && armed && !_prev_in_mode_armed;
	_prev_in_mode_armed = in_mode && armed;

	switch (_state) {
	case S::STATE_IDLE:
	case S::STATE_DONE:
	case S::STATE_ABORTED:
		if (_fly_requested.load()) {
			_fly_requested.store(false);

			if (!load_vehicle()) { _state = S::STATE_ABORTED; CC_ABORT("vehicle/mixer params"); break; }

			_use_ground_truth = _requested_ground_truth;
			_handing_back = false;

			if (_use_ground_truth && (_truth_src == 1 ? _visual_odometry.timestamp : _gt_attitude.timestamp) == 0) {
				_state = S::STATE_ABORTED;
				CC_ABORT("ground truth requested but not published (SIH only)");
				break;
			}

			_state = S::STATE_ENGAGING;
			_engage_start = now;
			_last_command = 0;

		} else if (switched_in) {
			// Switched to this mode with the RC while armed (from Stabilized): hold here and wait for fly
			// commands. Rig: level at the rig thrust, attitude only; free flight: this position.
			if (!load_vehicle()) { _state = S::STATE_ABORTED; CC_ABORT("vehicle/mixer params"); break; }

			V3 p, v, w;
			Quat q;
			read_state(rates, p, v, q, w);
			const M3 R = to_rotation_matrix(q);
			_requested = capable::Scenario::Options{};
			_requested.ref_only = _rig;
			_use_ground_truth = false;
			_virt_init = false;
			_hold_p = p;
			_hold_yaw = atan2(R(1, 0), R(0, 0));
			_home = V3(p(0), p(1), 0.0);
			_p0 = _home;
			_controller.reset();
			_controller.reset_dob();   // fresh engagement: nothing learnt yet
			_comp_init = false;
			_sysid_t0 = -1.0;
			_last_sample = 0;
			_engaged_at = now;
			_t_fly0 = now;
			_handing_back = false;
			// collective of the stock controller just before the switch: the ramp (rig) or blend (free flight) starts from it
			actuator_motors_s am{};
			_engage_thrust = 0.0;
			float u_stock[4] {};

			if (_actuator_motors_sub.copy(&am)) {
				for (int i = 0; i < 4; i++) {
					if (PX4_ISFINITE(am.control[i])) {
						u_stock[i] = am.control[i];
						_engage_thrust += fmax(0.0, (double)am.control[i]);
					}
				}
			}

			_engage_thrust *= (double)_f_max;
			// PX4's allocator has stopped: repeat its last command for this tick, the hold controller runs from the next
			publish_motors(u_stock);
			_state = S::STATE_HOLD;
			mavlink_vasprintf(_MSG_PRIO_INFO, &_mavlink_log_pub, "Capable Control: holding %s, ready for fly commands",
					  _rig ? "level" : "position");
			PX4_INFO("CAPABLE_HOLD: switched in, holding %s, yaw %.1f deg, from %.1f N; ready for fly commands",
				 _rig ? "level (rig, attitude only)" : "this position", _hold_yaw * 57.29578, _engage_thrust);

		} else if (in_mode && armed && now - _last_command > 500_ms) {
			// In this mode but not flying it (the switch-in failed): nothing drives the motors. Free flight: ask for
			// Position mode (read live: _rig is only latched by a successful load).
			int32_t rig = 0;
			param_get(param_find("CC_RIG_EN"), &rig);

			if (rig != 1) {
				_last_command = now;
				send_command(vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE, (float)vehicle_status_s::NAVIGATION_STATE_POSCTL);
			}
		}

		break;

	case S::STATE_ENGAGING:
		if (in_mode) { publish_motors(zero); }

		if (in_mode && armed) {
			V3 p0, v0, w0;
			Quat q0;
			read_state(rates, p0, v0, q0, w0);
			const M3 R0 = to_rotation_matrix(q0);
			const double yaw0 = atan2(R0(1, 0), R0(0, 0));
			_scenario.build(_requested, p0, yaw0);
			_engage_thrust = -1.0;
			_from_ground = true;
			_controller.reset_dob();   // from the ground: nothing learnt yet
			_comp_init = false;
			_sysid_t0 = -1.0;
			_engaged_at = now;
			_virt_init = false;

			if (_requested.virt) { virtual_init(p0, V3(), 0.0, p0(2)); }

			_p0 = p0;
			_home = p0;
			_last_pursuit_yaw = yaw0;
			_last_pursuit_acmd.zero();
			_controller.reset();
			_t_fly0 = now;
			_last_sample = 0;
			_state = S::STATE_FLY;
			mavlink_vasprintf(_MSG_PRIO_INFO, &_mavlink_log_pub, "Capable Control: %s started (%.0f s)",
					  scenario_name(_requested.type), _scenario.duration());
			PX4_INFO("CAPABLE_FLY: scenario %d, duration %.2f s, p0 (%.2f %.2f %.2f), yaw0 %.1f deg, rate ff %s, state %s",
				 (int)_requested.type, _scenario.duration(), p0(0), p0(1), p0(2), yaw0 * 57.29578,
				 _rate_ff ? "on" : "off", _use_ground_truth ? "ground truth" : "EKF");

			if (_requested.type == capable::Scenario::CIRCLE) {
				PX4_INFO("CAPABLE_CIRCLE: R %.3f m, rate %.4f rad/s, center (%.3f %.3f %.3f)",
					 _scenario.radius(), _scenario.rate(), _scenario.center()(0), _scenario.center()(1),
					 _scenario.center()(2));
			}

		} else if (now - _engage_start > 30_s) {
			_state = S::STATE_ABORTED;
			CC_ABORT("could not engage mode and arm");

		} else if (now - _last_command > 1_s) {
			_last_command = now;
			send_command(vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE, (float)_mode_id);
			send_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
		}

		break;

	case S::STATE_FLY: {
			if (!in_mode || !armed) {
				_state = S::STATE_ABORTED;
				CC_ABORT("left mode or disarmed in flight (nav_state %d, arming %d)",
					 (int)_vehicle_status.nav_state, (int)_vehicle_status.arming_state);
				break;
			}

			const double t_fly = (now - _t_fly0) * 1e-6;
			V3 p, v, w;
			Quat q;
			read_state(rates, p, v, q, w);

			if (const char *why = state_stale(now)) {
				if (!_rig) {
					hand_back(p, q);
					publish_motors(_last_u);
					CC_ABORT("%s: handing back to Position mode", why);
					break;
				}

				publish_motors(zero); _state = S::STATE_DISARMING; _last_command = 0;
				CC_ABORT("%s", why);
				break;
			}

			if (const char *why = rig_violation(q, w)) {
				publish_motors(zero); _state = S::STATE_DISARMING; _last_command = 0;
				CC_ABORT("rig limit: %s", why);
				break;
			}

			const V3 p_meas = p;

			if (_requested.virt && _virt_init) { p = _vp; v = _vv; }   // Stage B: (5.8) sees the virtual vehicle

			// Pursuit phase over: brake out of the current state and land.
			const bool pursuit_now = _scenario.in_pursuit(t_fly) && t_fly < _scenario.pursuit_end();

			if (_scenario.in_pursuit(t_fly) && !pursuit_now) {
				_scenario.append_brake_and_land(t_fly, p, v, _last_pursuit_acmd, _last_pursuit_yaw, _p0,
								_requested.type == capable::Scenario::DIVE ? 1.0 : 0.5);
				PX4_INFO("CAPABLE_PURSUIT_END: t_fly %.3f s, speed %.2f m/s", t_fly, v.norm());
			}

			if (t_fly >= _scenario.duration()) {
				PX4_INFO("CAPABLE_COMPLETE: t_fly %.3f s", t_fly);

				if (_requested.land) {
					publish_motors(zero);
					_state = S::STATE_DISARMING;
					_last_command = 0;
					end_test(true, "%s complete (%.1f s), landed: disarming", scenario_name(_requested.type), t_fly);

				} else if (!_requested.ref_only && (p - _scenario.ref(_scenario.duration()).p).norm() > 5.0) {
					// (attitude-only -F flights have no position loop: their position is not checked)
					// Did not end near its end point (crashed / ran away): do not chain the next test from here.
					// Free flight: hold here and hand back to PX4; the rig stops.
					if (!_rig) {
						const double miss = (p - _scenario.ref(_scenario.duration()).p).norm();
						hand_back(p_meas, q);
						CC_ABORT("ended %.1f m from the end point: holding, handing back to Position mode", miss);
						publish_motors(_last_u);   // HOLD takes over from the next tick
						break;
					}

					publish_motors(zero);
					_state = S::STATE_DISARMING;
					_last_command = 0;
					CC_ABORT("ended %.1f m from the end point, not holding",
						 (p - _scenario.ref(_scenario.duration()).p).norm());

				} else {
					const capable::Ref end = _scenario.ref(_scenario.duration());
					_hold_p = end.p;
					_hold_yaw = end.yaw;
					_last_sample = 0;
					_state = S::STATE_HOLD;
					PX4_INFO("CAPABLE_HOLD: hovering at (%.2f %.2f %.2f), ready for the next fly request",
						 _hold_p(0), _hold_p(1), _hold_p(2));
					end_test(true, "%s complete (%.1f s), holding %s, ready for the next command",
						 scenario_name(_requested.type), t_fly, _requested.ref_only ? "level" : "position");
					publish_motors(_last_u);   // HOLD takes over from the next tick
				}

				break;
			}

			const double dt = _last_sample ? (now - _last_sample) * 1e-6 : 0.0;
			_last_sample = now;

			// Read once: the pursuit branches below replace it, the rest re-read the same value, and the
			// phase is what decides whether the observer may learn this tick.
			capable::Ref ref = _scenario.ref(t_fly);
			const hrt_abstime t_start = hrt_absolute_time();
			V3 a_raw;
			ControlOutput out;

			// (6) observer. dt is passed separately from the rate feed-forward's, so fly -n does not also
			// switch the observer off; a stale or missing sample freezes it instead of feeding it garbage.
			V3 a_meas{};
			const bool have_accel = _dob_en && read_accel(now, q, a_meas);
			const bool dob_freeze = dob_frozen(pursuit_now ? (uint8_t)S::PHASE_PURSUIT : ref.phase);
			const V3 *a_meas_p = have_accel ? &a_meas : nullptr;
			_controller.dob_freeze(dob_freeze);

			if (pursuit_now && _requested.type == capable::Scenario::ACCEL) {
				// Closed-loop acceleration test: a fixed command in the world frame, heading held, no
				// position loop. What (6) has to hold on its own.
				a_raw = _requested.accel_cmd;
				ref.p = p; ref.v = v; ref.a = a_raw;
				ref.yaw = _last_pursuit_yaw;
				ref.phase = S::PHASE_ACCEL;
				out = _controller.attitude_and_thrust(a_raw, q, w, ref.yaw, &v, _rate_ff ? dt : 0.0, a_meas_p, dt);
				_last_pursuit_acmd = a_raw;

			} else if (pursuit_now && _requested.type == capable::Scenario::DIVE) {
				// Sec. V-J test: a fixed acceleration command (lat_g north, down_g down), fixed yaw,
				// no position loop. Inverting regime rolls over to thrust down; upright caps at free fall.
				a_raw = V3(_requested.lat_g * GRAVITY, 0.0, -_requested.down_g * GRAVITY);
				ref.p = p; ref.v = v; ref.a = a_raw;
				ref.yaw = _last_pursuit_yaw;
				ref.phase = S::PHASE_DIVE;
				out = _controller.attitude_and_thrust(a_raw, q, w, ref.yaw, &v, _rate_ff ? dt : 0.0, a_meas_p, dt);
				_last_pursuit_acmd = a_raw;

			} else if (pursuit_now) {
				// compute_pursuit: a pure horizontal acceleration command perpendicular to the current
				// velocity (turning left), zero vertical, yaw along the velocity. No position loop.
				const double sp = sqrt(v(0) * v(0) + v(1) * v(1));
				const V3 n = sp > 0.1 ? V3(-v(1) / sp, v(0) / sp, 0.0) : V3(0.0, 0.0, 0.0);
				// Entry ramp (option -R, test of the entry-transient hypothesis; 0 = step as in the paper)
				const double tp = t_fly - _scenario.pursuit_start(), R = _requested.pursuit_ramp;
				const double x = R > 0.0 ? fmin(1.0, tp / R) : 1.0;
				a_raw = n * (_requested.g_level * GRAVITY * x * x * (3.0 - 2.0 * x));
				ref.p = p; ref.v = v; ref.a = a_raw;
				ref.yaw = sp > 0.1 ? atan2(v(1), v(0)) : _last_pursuit_yaw;
				ref.phase = S::PHASE_PURSUIT;
				out = _controller.attitude_and_thrust(a_raw, q, w, ref.yaw, &v, _rate_ff ? dt : 0.0, a_meas_p, dt);
				_last_pursuit_acmd = a_raw;
				_last_pursuit_yaw = ref.yaw;

			} else if (_requested.ref_only) {
				// Stage A (rig): no position loop. (5.8) is bypassed: a_cmd = a_ref, and the drag
				// feed-forward of (5.2) uses v_ref. (5.1) to (5.7) run on the real attitude and rates.
				a_raw = ref.a;
				out = _controller.attitude_and_thrust(ref.a, q, w, ref.yaw, &ref.v, _rate_ff ? dt : 0.0, a_meas_p, dt);
				_last_pursuit_yaw = ref.yaw;

			} else {
				out = _controller.compute(p, v, q, w, ref.p, ref.v, ref.a, ref.yaw, _rate_ff ? dt : 0.0, &a_raw,
							  a_meas_p, dt);
				_last_pursuit_yaw = ref.yaw;
			}

			V3 tau_inj{};

			if (ref.phase == S::PHASE_SYSID) {
				if (_sysid_t0 < 0.0) { _sysid_t0 = t_fly; }

				tau_inj = sysid_injection(t_fly - _sysid_t0);
			}

			if (_requested.virt && _virt_init && dt > 0.0) { virtual_step(q, out.collective_thrust, dt); }

			V3 tau_cmd = compensate(out.body_torque, w, dt) + tau_inj;
			double thrust_cmd = out.collective_thrust;
			float out_scale;
			bool thr_capped;
			limit_output(thrust_cmd, tau_cmd, (now - _engaged_at) * 1e-6, out_scale, thr_capped);
			float u[4], u_raw[4];
			bool sat_mixer;
			mix(thrust_cmd, to_frd(tau_cmd), u, u_raw, sat_mixer);
			// The collective that limit_output() actually sent (the engage blend moves it for CC_ENG_RAMP
			// seconds) is what the next a_model must be built from, and this vehicle's own mixer rail is a
			// saturation like the paper plant's.
			_controller.dob_post(thrust_cmd, sat_mixer);

			if (ref.phase == S::PHASE_TOUCHDOWN) { for (float &x : u) { x = 0.f; } }

			publish_motors(u);
			publish_setpoints(out.desired_quaternion, out.omega_desired, thrust_cmd, _requested.ref_only ? nullptr : &ref);
			const uint32_t compute_us = (uint32_t)(hrt_absolute_time() - t_start);
			publish_status(now, t_fly, dt, pursuit_now, p, v, q, w, ref, a_raw, out, tau_cmd, tau_inj, u, u_raw, sat_mixer,
				       out_scale, thr_capped, a_meas, dob_freeze || !have_accel, compute_us);
			break;
		}

	case S::STATE_HOLD: {
			// Hover at the end point of the last scenario (position loop on, logged as PHASE_HOLD_IDLE) until the
			// next fly request, which starts from here instead of from the ground.
			if (!in_mode || !armed) {
				_state = S::STATE_ABORTED;

				if (_handing_back) {
					_handing_back = false;
					PX4_INFO("CAPABLE_HANDBACK: PX4 has the vehicle (nav_state %d, arming %d)",
						 (int)_vehicle_status.nav_state, (int)_vehicle_status.arming_state);
					break;
				}

				CC_ABORT("left mode or disarmed while holding");
				break;
			}

			if (_handing_back && now - _last_command > 500_ms) {
				_last_command = now;
				send_command(vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE, (float)vehicle_status_s::NAVIGATION_STATE_POSCTL);
			}

			V3 p, v, w;
			Quat q;
			read_state(rates, p, v, q, w);

			if (const char *why = state_stale(now)) {
				if (!_rig) {
					if (!_handing_back) {
						hand_back(_hold_p, q);
						CC_ABORT("%s: handing back to Position mode", why);
					}

					publish_motors(_last_u);   // nothing better to fly on until PX4 takes over
					break;
				}

				publish_motors(zero); _state = S::STATE_DISARMING; _last_command = 0;
				CC_ABORT("%s", why);
				break;
			}

			if (const char *why = rig_violation(q, w)) {
				publish_motors(zero); _state = S::STATE_DISARMING; _last_command = 0;
				CC_ABORT("rig limit: %s", why);
				break;
			}

			const bool hold_virtual = _requested.virt && _virt_init;   // flag of the flight that just ended

			if (hold_virtual) { p = _vp; v = _vv; }

			if (_fly_requested.load() && _handing_back) {
				_fly_requested.store(false);   // custom_command refuses it too; the vehicle is going back to PX4

			} else if (_fly_requested.load()) {
				_fly_requested.store(false);

				if (!load_vehicle()) { _state = S::STATE_ABORTED; CC_ABORT("vehicle/mixer params"); break; }

				_use_ground_truth = _requested_ground_truth;
				read_state(rates, p, v, q, w);

				if (!_requested.virt) {
					_virt_init = false;

				} else if (_virt_init) {
					p = _vp; v = _vv;              // continue the virtual flight

				} else {
					virtual_init(p, v, _mass * GRAVITY, _home(2));   // start it from the real state, hovering
				}

				const M3 R0 = to_rotation_matrix(q);
				const double yaw0 = atan2(R0(1, 0), R0(0, 0));
				_scenario.build(_requested, _home, yaw0, &p);
				_from_ground = false;   // started from a hover: no ground contact, PHASE_TAKEOFF is a transit
				_sysid_t0 = -1.0;
				_p0 = _home;
				_last_pursuit_yaw = yaw0;
				_last_pursuit_acmd.zero();
				_controller.reset();
				_t_fly0 = now;
				_last_sample = 0;
				_state = S::STATE_FLY;
				mavlink_vasprintf(_MSG_PRIO_INFO, &_mavlink_log_pub, "Capable Control: %s started (%.0f s)",
						  scenario_name(_requested.type), _scenario.duration());
				PX4_INFO("CAPABLE_FLY: scenario %d, duration %.2f s, p0 (%.2f %.2f %.2f), yaw0 %.1f deg, rate ff %s, state %s, from hover (%.2f %.2f %.2f)",
					 (int)_requested.type, _scenario.duration(), _home(0), _home(1), _home(2), yaw0 * 57.29578,
					 _rate_ff ? "on" : "off", _use_ground_truth ? "ground truth" : "EKF", p(0), p(1), p(2));

				if (_requested.type == capable::Scenario::CIRCLE) {
					PX4_INFO("CAPABLE_CIRCLE: R %.3f m, rate %.4f rad/s, center (%.3f %.3f %.3f)",
						 _scenario.radius(), _scenario.rate(), _scenario.center()(0), _scenario.center()(1),
						 _scenario.center()(2));
				}

				publish_motors(_last_u);   // FLY takes over from the next tick
				break;
			}

			const double dt = _last_sample ? (now - _last_sample) * 1e-6 : 0.0;
			_last_sample = now;
			const hrt_abstime t_start = hrt_absolute_time();
			// After an attitude-only (-F) flight, hold level at hover thrust: no position to hold on the rig.
			const V3 vzero{};
			V3 a_raw{};
			// (6) runs while holding too: hovering is where a mass or thrust-model error shows up as a
			// steady d_hat, and the estimate is then already learnt when the next fly starts.
			V3 a_meas{};
			const bool have_accel = _dob_en && read_accel(now, q, a_meas);
			const bool dob_freeze = dob_frozen((uint8_t)S::PHASE_HOLD_IDLE);
			const V3 *a_meas_p = have_accel ? &a_meas : nullptr;
			_controller.dob_freeze(dob_freeze);
			const ControlOutput out = _requested.ref_only
						  ? _controller.attitude_and_thrust(V3(), q, w, _hold_yaw, &vzero, _rate_ff ? dt : 0.0,
								  a_meas_p, dt)
						  : _controller.compute(p, v, q, w, _hold_p, V3(), V3(), _hold_yaw, _rate_ff ? dt : 0.0, &a_raw,
								  a_meas_p, dt);

			if (hold_virtual && dt > 0.0) { virtual_step(q, out.collective_thrust, dt); }

			V3 tau_cmd = compensate(out.body_torque, w, dt);
			double thrust_cmd = out.collective_thrust;
			float out_scale;
			bool thr_capped;
			limit_output(thrust_cmd, tau_cmd, (now - _engaged_at) * 1e-6, out_scale, thr_capped);
			float u[4], u_raw[4];
			bool sat_mixer;
			mix(thrust_cmd, to_frd(tau_cmd), u, u_raw, sat_mixer);
			_controller.dob_post(thrust_cmd, sat_mixer);
			publish_motors(u);
			capable::Ref hold_ref{};
			hold_ref.p = _hold_p; hold_ref.yaw = _hold_yaw; hold_ref.phase = S::PHASE_HOLD_IDLE;
			publish_setpoints(out.desired_quaternion, out.omega_desired, thrust_cmd, _requested.ref_only ? nullptr : &hold_ref);
			const uint32_t compute_us = (uint32_t)(hrt_absolute_time() - t_start);
			publish_status(now, (now - _t_fly0) * 1e-6, dt, false, p, v, q, w, hold_ref, a_raw, out, tau_cmd, V3(), u, u_raw,
				       sat_mixer, out_scale, thr_capped, a_meas, dob_freeze || !have_accel, compute_us);
			break;
		}

	case S::STATE_DISARMING:
		if (in_mode) { publish_motors(zero); }

		if (!armed) {
			_state = S::STATE_DONE;
			PX4_INFO("CAPABLE_DONE: disarmed");

		} else if (now - _last_command > 500_ms) {
			_last_command = now;
			send_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.f, 21196.f);   // forced: sim only
		}

		break;
	}
}

// ---------------------------------------------------------------- module boilerplate

int CapableControl::task_spawn(int argc, char *argv[])
{
	CapableControl *instance = new CapableControl();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) { return PX4_OK; }

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int CapableControl::custom_command(int argc, char *argv[])
{
	if (!is_running(desc)) { return print_usage("not running"); }

	if (argc >= 2 && !strcmp(argv[0], "fly")) {
		CapableControl *self = get_instance<CapableControl>(desc);
		capable::Scenario::Options o{};
		bool rate_ff = true;
		bool ground_truth = false;
		double sysid_scale = 1.0;

		if (!strcmp(argv[1], "hover")) { o.type = capable::Scenario::HOVER; }

		else if (!strcmp(argv[1], "translate")) { o.type = capable::Scenario::TRANSLATE; }

		else if (!strcmp(argv[1], "circle")) { o.type = capable::Scenario::CIRCLE; }

		else if (!strcmp(argv[1], "pursuit")) { o.type = capable::Scenario::PURSUIT; }

		else if (!strcmp(argv[1], "sysid")) { o.type = capable::Scenario::SYSID; }

		else if (!strcmp(argv[1], "tilt")) { o.type = capable::Scenario::TILT; o.turn_time = 2.0; }

		else if (!strcmp(argv[1], "dive")) { o.type = capable::Scenario::DIVE; o.turn_time = 1.0; o.altitude = 100.0; }

		else if (!strcmp(argv[1], "accel")) { o.type = capable::Scenario::ACCEL; o.turn_time = 3.0; o.altitude = 30.0; }

		else { return print_usage("unknown scenario"); }

		int myoptind = 2, ch;
		const char *myoptarg = nullptr;

		if (o.type == capable::Scenario::ACCEL) {
			// fly accel ax,ay,az: the command itself, in m/s^2 and NWU, as a positional argument before the
			// options. Not in g, unlike -g / -d / -l, so it matches the Python bench's --accel. Commas and
			// spaces separate equally, so "5,0,0", "5, 0, 0" and "5 0 0" all work.
			double a[3] {};
			int got = 0;

			while (got < 3 && myoptind < argc) {
				const char *s = argv[myoptind];

				// A leading '-' is an option unless a digit or a point follows it: no option is a number,
				// so "-2" is the component and "-T" ends the vector.
				if (s[0] == '-' && !((s[1] >= '0' && s[1] <= '9') || s[1] == '.')) { break; }

				while (got < 3 && *s != '\0') {
					char *end = nullptr;
					const double value = strtod(s, &end);

					if (end == s) { break; }

					a[got++] = value;
					s = end;

					while (*s == ',' || *s == ' ') { s++; }
				}

				myoptind++;
			}

			if (got != 3) {
				return print_usage("fly accel needs ax,ay,az in m/s^2, e.g. fly accel 5,0,0 -T 3");
			}

			o.accel_cmd = V3(a[0], a[1], a[2]);
		}

		while ((ch = px4_getopt(argc, argv, "g:v:r:y:a:T:d:l:R:A:ntLFV", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'd': o.down_g = atof(myoptarg); break;

			case 'l': o.lat_g = atof(myoptarg); break;

			case 'T': o.turn_time = atof(myoptarg); break;

			case 'R': o.pursuit_ramp = atof(myoptarg); break;

			case 'A': sysid_scale = atof(myoptarg); break;

			case 'g': o.g_level = atof(myoptarg); break;

			case 'v': o.speed = atof(myoptarg); break;

			case 'r': o.revs = atof(myoptarg); break;

			case 'a': o.altitude = atof(myoptarg); break;

			case 'y': o.velocity_yaw = strcmp(myoptarg, "fixed") != 0; break;

			case 'n': rate_ff = false; break;

			case 't': ground_truth = true; break;

			case 'L': o.land = true; break;

			case 'F': o.ref_only = true; break;

			case 'V': o.virt = true; break;

			default: return print_usage("bad option");
			}
		}

		if (self->_state == capable_control_status_s::STATE_ENGAGING || self->_state == capable_control_status_s::STATE_FLY) {
			return print_usage("already flying");
		}

		int32_t rig = 0;
		param_get(param_find("CC_RIG_EN"), &rig);

		if (o.type == capable::Scenario::ACCEL) {
			if (o.ref_only || o.virt) {
				PX4_ERR("fly accel is a free-flight test: -F and -V fly a reference or a virtual vehicle, not this command");
				return 1;
			}

			if (o.turn_time <= 0.0) {
				PX4_ERR("fly accel: -T must be greater than 0 (the time the command is held)");
				return 1;
			}

			float acc_max_g = 4.f;
			param_get(param_find("CC_ACC_MAX_G"), &acc_max_g);
			const double mag = o.accel_cmd.norm(), T = o.turn_time;

			if (mag > (double)acc_max_g * GRAVITY) {
				PX4_ERR("|a| %.2f m/s^2 is above the CC_ACC_MAX_G envelope (%.2f m/s^2): (5.1) would clip it",
					mag, (double)acc_max_g * GRAVITY);
				return 1;
			}

			// What the vehicle will actually do, so the box can be checked before it is flown: the command
			// phase, then append_brake_and_land()'s quintic at 0.5 g.
			const double v_end = mag * T, d_cmd = 0.5 * mag * T * T;
			const double t_brake = fmax(3.0, 1.5 * v_end / (0.5 * GRAVITY));
			PX4_INFO("CAPABLE_ACCEL: (%.2f %.2f %.2f) m/s^2 = %.2f g for %.1f s -> %.1f m/s, %.0f m; then a %.1f s brake, about %.0f m more",
				 o.accel_cmd(0), o.accel_cmd(1), o.accel_cmd(2), mag / GRAVITY, T, v_end, d_cmd, t_brake,
				 v_end * t_brake * 0.5);
		}

		if (o.type == capable::Scenario::TILT && !o.ref_only && !o.virt) {
			PX4_ERR("tilt is a rig test: fly tilt -F (attitude only) or -V; in free flight it would reach about 50 m/s");
			return 1;
		}

		if (rig == 1 && !o.ref_only && !o.virt) {
			PX4_ERR("rig mode (CC_RIG_EN 1) has no position: use fly -F (attitude only) or -V (virtual translation)");
			return 1;
		}

#ifdef __PX4_NUTTX

		// Hardware: the pilot engages the mode (RC switch, armed); fly never arms or switches mode by itself.
		// (SITL keeps the automatic engage and arm of the test runners.)
		if (self->_state != capable_control_status_s::STATE_HOLD) {
			PX4_ERR("not engaged: arm (Stabilized), switch to Capable Control on the RC, then fly");
			return 1;
		}

		// Free flight: -F and -V have no position loop, the vehicle would fly away at the reference acceleration.
		if (rig != 1 && (o.ref_only || o.virt)) {
			PX4_ERR("-F and -V are rig tests (CC_RIG_EN 1): no position loop in free flight");
			return 1;
		}

#endif

		if (self->_handing_back) {
			PX4_ERR("handing back to PX4 after an abort: switch out of and back into Capable Control on the RC first");
			return 1;
		}

		self->_requested = o;
		self->_rate_ff = rate_ff;
		self->_requested_ground_truth = ground_truth;
		self->_sysid_scale = sysid_scale;
		self->_fly_requested.store(true);
		PX4_INFO("fly request accepted");
#ifdef __PX4_NUTTX
		return self->wait_for_test();
#else
		return 0;   // SITL runners follow the console log
#endif
	}

	return print_usage("unknown command");
}

int CapableControl::wait_for_test()
{
#ifdef __PX4_NUTTX
	// Hardware console: wait for the end of the test and print it. Any key stops waiting (the test continues).
	using S = capable_control_status_s;
	const int seq0 = _end_seq.load();
	const hrt_abstime t0 = hrt_absolute_time();
	hrt_abstime last = t0;
	PX4_INFO_RAW("running; prints when it ends (press Enter to stop waiting, the test continues)\n");

	while (true) {
		pollfd fds{};
		fds.fd = 0;
		fds.events = POLLIN;

		if (poll(&fds, 1, 200) > 0) {
			char c;
			(void)read(0, &c, 1);
			PX4_INFO_RAW("stopped waiting; the test continues (capable_control status)\n");
			return 0;
		}

		if (_end_seq.load() != seq0) {
			PX4_INFO_RAW("%s: %s\n", _end_ok ? "DONE" : "ABORTED", _end_msg);
			return _end_ok ? 0 : 1;
		}

		const hrt_abstime now = hrt_absolute_time();

		if (_state == S::STATE_FLY) {
			if (now - last > 5_s) {
				last = now;
				PX4_INFO_RAW("  %s: %.0f of %.0f s\n", scenario_name(_requested.type), (double)((now - _t_fly0) * 1e-6),
					     _scenario.duration());
			}

		} else if (now - t0 > 5_s) {
			PX4_INFO_RAW("not running (state %d)\n", (int)_state);
			return 1;
		}
	}

#endif
	return 0;
}

int CapableControl::print_status()
{
	PX4_INFO("mode nav_state %d, arming check %d, state %d, mixer %s (Fmax %.2f N, %s thrust curve)",
		 _mode_id, _arming_check_id, (int)_state, _mixer_ok ? "ok" : "not loaded", (double)_f_max,
		 _thr_model ? "quadratic" : "linear");

	for (int i = 0; i < 4; i++) {
		PX4_INFO("  rotor %d: px %.4f py %.4f km %.4f", i, (double)_rotor_px[i], (double)_rotor_py[i], (double)_rotor_km[i]);
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_interval_perf);
	return 0;
}

int CapableControl::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Standalone flight mode running the geometric high-authority controller of MATH_Rishabh.pdf
from state estimate to motor commands. No PX4 control loop or control allocator runs in this mode.

### Examples
$ capable_control start
$ capable_control fly hover
$ capable_control fly circle -g 4 -v 20 -r 3 -y velocity
$ capable_control fly accel 5,0,0 -T 3 -a 30

### Acceleration test
`fly accel ax,ay,az` holds a fixed acceleration command for -T seconds with NO position loop, then
brakes and hovers. The vector is in m/s^2 (not g) in the controller's NWU frame, gravity excluded, so
0,0,0 commands "keep the current velocity" -- which with CC_DOB_EN 0 drifts at g (m_model/m_true - 1).
Set CC_DOB_EN 1 for the closed-loop version.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("capable_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fly", "engage the mode, arm and fly a scenario: hover|translate|circle|pursuit|dive|sysid|tilt|accel (tilt: rig only, with -F or -V; accel: takes ax,ay,az [m/s^2] before the options)");
	PRINT_MODULE_USAGE_PARAM_FLOAT('g', 1.f, 0.1f, 10.f, "circle: lateral acceleration [g]", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('v', 20.f, 1.f, 60.f, "circle: speed [m/s]", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', 3.f, 0.5f, 20.f, "circle: steady revolutions", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('a', 20.f, 2.f, 200.f, "circle/pursuit/accel: altitude [m] (accel: default 30, measured from the take-off point)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('T', 10.f, 1.f, 120.f, "pursuit: duration of the pursuit turn [s]; tilt: ramp time [s] (default 2); accel: hold time of the command [s] (default 3)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('A', 1.f, 0.1f, 4.f, "sysid: excitation amplitude scale (1 = 0.5 N m roll/pitch, 0.15 N m yaw)", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('R', 0.f, 0.f, 5.f, "pursuit: entry ramp of the 4 g command [s] (0 = step, paper)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('y', "velocity", "velocity|fixed", "circle: yaw policy", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('d', 2.f, 0.f, 6.f, "dive: commanded downward acceleration [g]", true);
	PRINT_MODULE_USAGE_PARAM_FLOAT('l', 0.f, 0.f, 6.f, "dive: commanded lateral (north) acceleration [g]", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('n',"disable the attitude-rate feed-forward (diagnostic)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('V', "virtual translation: (5.8) flies a point mass driven by the real attitude (rig Stage B)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('F', "attitude-only: bypass (5.8), a_cmd = reference acceleration (rig Stage A)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('L', "return and land at the end (default: brake and hover, ready for the next fly)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "feed SIH ground-truth state instead of the EKF (perfect state feedback)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int capable_control_main(int argc, char *argv[])
{
	return ModuleBase::main(CapableControl::desc, argc, argv);
}
