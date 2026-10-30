/**
 * @file CapableFlightController.hpp
 *
 * Line-by-line C++ port of capable_flight_controller.py (MATH_Rishabh.pdf, Sec. V).
 * Kept deliberately literal -- same frames (world z-up, body FLU), same order of operations,
 * double precision -- so it can be checked tick-by-tick against the Python original by
 * replaying logged inputs through it (see px4_offboard/verify_port.py).
 */

#pragma once

#include <cmath>
#include <matrix/matrix/math.hpp>

namespace capable
{

using V3 = matrix::Vector3d;
using M3 = matrix::SquareMatrix<double, 3>;

static constexpr double GRAVITY = 9.80665;

// ---------------------------------------------------------------- quaternion helpers (quaternion_utils.py)

struct Quat {
	double w{1.0}, x{0.0}, y{0.0}, z{0.0};
	V3 vec() const { return V3(x, y, z); }
};

inline Quat normalize(const Quat &q)
{
	const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

	if (n < 1e-12) { return Quat{}; }

	return Quat{q.w / n, q.x / n, q.y / n, q.z / n};
}

inline Quat conjugate(const Quat &q) { return Quat{q.w, -q.x, -q.y, -q.z}; }

inline Quat multiply(const Quat &p, const Quat &q)
{
	return Quat{
		p.w *q.w - p.x *q.x - p.y *q.y - p.z * q.z,
		p.w *q.x + p.x *q.w + p.y *q.z - p.z * q.y,
		p.w *q.y - p.x *q.z + p.y *q.w + p.z * q.x,
		p.w *q.z + p.x *q.y - p.y *q.x + p.z * q.w};
}

inline M3 to_rotation_matrix(const Quat &q_in)
{
	const Quat q = normalize(q_in);
	const double w = q.w, x = q.x, y = q.y, z = q.z;
	M3 R;
	R(0, 0) = 1.0 - 2.0 * (y * y + z * z); R(0, 1) = 2.0 * (x * y - w * z);       R(0, 2) = 2.0 * (x * z + w * y);
	R(1, 0) = 2.0 * (x * y + w * z);       R(1, 1) = 1.0 - 2.0 * (x * x + z * z); R(1, 2) = 2.0 * (y * z - w * x);
	R(2, 0) = 2.0 * (x * z - w * y);       R(2, 1) = 2.0 * (y * z + w * x);       R(2, 2) = 1.0 - 2.0 * (x * x + y * y);
	return R;
}

/** Shepperd's method, canonical hemisphere w >= 0. */
inline Quat from_rotation_matrix(const M3 &R)
{
	const double trace = R(0, 0) + R(1, 1) + R(2, 2);
	const double c[4] = {trace, R(0, 0), R(1, 1), R(2, 2)};
	int branch = 0;

	for (int i = 1; i < 4; i++) { if (c[i] > c[branch]) { branch = i; } }   // first max, like np.argmax

	Quat q;

	if (branch == 0) {
		const double w = 0.5 * std::sqrt(fmax(1.0 + trace, 0.0));
		q = Quat{w, (R(2, 1) - R(1, 2)) / (4 * w), (R(0, 2) - R(2, 0)) / (4 * w), (R(1, 0) - R(0, 1)) / (4 * w)};

	} else if (branch == 1) {
		const double x = 0.5 * std::sqrt(fmax(1.0 + R(0, 0) - R(1, 1) - R(2, 2), 0.0));
		q = Quat{(R(2, 1) - R(1, 2)) / (4 * x), x, (R(0, 1) + R(1, 0)) / (4 * x), (R(0, 2) + R(2, 0)) / (4 * x)};

	} else if (branch == 2) {
		const double y = 0.5 * std::sqrt(fmax(1.0 - R(0, 0) + R(1, 1) - R(2, 2), 0.0));
		q = Quat{(R(0, 2) - R(2, 0)) / (4 * y), (R(0, 1) + R(1, 0)) / (4 * y), y, (R(1, 2) + R(2, 1)) / (4 * y)};

	} else {
		const double z = 0.5 * std::sqrt(fmax(1.0 - R(0, 0) - R(1, 1) + R(2, 2), 0.0));
		q = Quat{(R(1, 0) - R(0, 1)) / (4 * z), (R(0, 2) + R(2, 0)) / (4 * z), (R(1, 2) + R(2, 1)) / (4 * z), z};
	}

	q = normalize(q);
	return q.w < 0.0 ? Quat{-q.w, -q.x, -q.y, -q.z} : q;
}

inline V3 cross(const V3 &a, const V3 &b) { return a.cross(b); }

// ---------------------------------------------------------------- vehicle capability (vehicle_capability.py, Table II)

struct Capability {
	double mass;
	V3 inertia;
	double max_lateral_accel_g, max_speed, max_thrust_to_weight, max_motor_speed;
	double omega_n_att, zeta_att, omega_n_yaw, zeta_yaw;
	double linear_drag;
	double thrust_coefficient, torque_coefficient, arm_length;
	V3 kp_pos, kd_vel;
	bool upright{false};   // Sec. V-J: true = upright regime (5.9), false = inverting (shared controller)

	double max_lateral_accel() const { return max_lateral_accel_g * GRAVITY; }
	V3 kp_att() const
	{
		return V3(inertia(0) * omega_n_att * omega_n_att, inertia(1) * omega_n_att * omega_n_att,
			  inertia(2) * omega_n_yaw * omega_n_yaw);
	}
	V3 kd_rate() const
	{
		return V3(2.0 * zeta_att * omega_n_att * inertia(0), 2.0 * zeta_att * omega_n_att * inertia(1),
			  2.0 * zeta_yaw * omega_n_yaw * inertia(2));
	}
};

inline Capability chaser()
{
	Capability c{};
	c.mass = 3.0;
	c.inertia = V3(0.025, 0.025, 0.046);
	c.max_lateral_accel_g = 4.0;
	c.max_speed = 50.0;
	c.max_thrust_to_weight = 5.5;
	c.max_motor_speed = 2100.0;
	c.omega_n_att = 14.0; c.zeta_att = 0.7;
	c.omega_n_yaw = 6.0; c.zeta_yaw = 0.7;
	c.linear_drag = 0.18;
	c.thrust_coefficient = 44.13 / (2100.0 * 2100.0);
	c.torque_coefficient = 1.76 / (2.0 * 2100.0 * 2100.0);
	c.arm_length = 11.0 / 44.13;
	// NOT FROM THE PAPER (not in the available excerpt): w_n = 2 rad/s, zeta = 0.9, isotropic.
	c.kp_pos = V3(4.0, 4.0, 4.0);
	c.kd_vel = V3(3.6, 3.6, 3.6);
	return c;
}

// ---------------------------------------------------------------- controller (capable_flight_controller.py)

struct ControlOutput {
	double collective_thrust{0.0};
	V3 body_torque{};
	Quat desired_quaternion{};
	V3 desired_acceleration{};      // after saturation, incl. gravity compensation
	V3 commanded_acceleration{};    // after saturation, excl. gravity compensation
	double tilt_rad{0.0};
	bool saturated_accel{false};
	bool saturated_thrust{false};
	bool saturated_mixer{false};
	// exposed for logging (locals in the Python original)
	V3 e_r{};
	V3 omega_desired{};
	// (6) acceleration disturbance observer
	bool dob_active{false};             // the observer ran this tick (a_des had d_hat removed)
	V3 accel_disturbance{};             // d_hat, subtracted from a_des
	V3 accel_model{};                   // a_model, what the model says the vehicle should be doing
	double accel_disturbance_thrust{0.0};   // along-thrust part of d_hat (scalar on b_z)
};

class CapableFlightController
{
public:
	explicit CapableFlightController(const Capability &cap) : _cap(cap)
	{
		_max_thrust = cap.max_thrust_to_weight * cap.mass * GRAVITY;
	}

	/** Does NOT forget the disturbance estimate: a test started from a hover inherits what the hover
	 *  already learnt, instead of spending its first half-second re-learning the same mass error. Call
	 *  reset_dob() explicitly for a genuinely fresh start (mode entry, a flight from the ground). */
	void reset() { _have_prev_q_des = false; _omega_des_filtered.zero(); }

	const V3 &omega_des_filtered() const { return _omega_des_filtered; }
	double max_thrust() const { return _max_thrust; }

	// ------------------------------------------------------------ (6) acceleration disturbance observer
	//
	// Port of capable_udan_controller.py::_update_dob (Capable_python_testing). Everything above is
	// feed-forward: the commanded acceleration is delivered only if mass, thrust coefficient and drag are
	// exactly what the controller assumes. The observer compares the measured acceleration with the model
	// evaluated at the vehicle's ACTUAL attitude and lagged thrust, so the normal tilting transient appears
	// in both terms and cancels; only model error (thrust, mass, drag, wind) is left to remove. Feeding back
	// the command error (a_cmd - a_meas) instead would ask for ~2x at a step, because a_meas is still ~0
	// while the vehicle tilts.
	//
	//     a_model = R(q) e3 T_hat/m - g e3 - k_D v
	//     e       = LPF(a_meas) - LPF(a_model)        the same filter on both, so they stay in phase
	//     d_hat   = LPF(e . b_z) b_z + LPF(e_perp)    split along / across the thrust axis
	//     a_des   = a_cmd + g e3 + k_D v - d_hat
	//
	// Thrust and mass errors act ALONG the thrust axis, so that part is kept as a scalar on b_z and turns
	// with the vehicle instead of lagging behind a world-frame filter. Each channel has its own clip.

	void configure_dob(bool enable, double tau, double meas_tau, double max_g, double thrust_frac,
			   double motor_tau)
	{
		_dob_en = enable;
		_dob_tau = fmax(tau, 1e-3);
		_dob_meas_tau = fmax(meas_tau, 1e-4);
		_dob_max = fmax(max_g, 0.0) * GRAVITY;
		_dob_thrust_frac = fmax(thrust_frac, 0.0);
		_dob_mot_tau = fmax(motor_tau, 1e-3);
		reset_dob();
	}

	bool dob_enabled() const { return _dob_en; }

	/** The learnt estimate, so it can survive the controller being rebuilt when a fly reloads the
	 *  parameters. Only the two filtered states are carried: T_hat and the measurement filters re-seed
	 *  themselves on the next tick. */
	void get_dob_estimate(V3 &d_perp, double &d_thrust) const { d_perp = _dob_d_perp; d_thrust = _dob_d_thrust; }
	void set_dob_estimate(const V3 &d_perp, double d_thrust)
	{
		_dob_d_perp = d_perp;
		_dob_d_thrust = d_thrust;
		_dob_d = d_perp;   // the b_z part is added back by the next update, on the current b_z
	}

	/** Forget the disturbance estimate (call when (re)starting a flight). */
	void reset_dob()
	{
		_dob_d.zero(); _dob_d_perp.zero(); _dob_meas_f.zero(); _dob_model_f.zero(); _dob_model.zero();
		_dob_d_thrust = 0.0;
		_dob_thrust = -1.0;
		_dob_prev_thrust_cmd = -1.0;
		_dob_filt_init = false;
		_dob_hold = false;
	}

	/** Stop the estimate moving without forgetting it: on the ground, on the rig, or with no usable
	 *  measurement. The ground reaction force and a pivot are not model error. */
	void dob_freeze(bool freeze) { _dob_freeze = freeze; }

	/** Close the tick: the thrust actually sent to the mixer (which the module may have ramped or capped
	 *  after the controller returned) is what the next a_model must be built from, and a saturated tick is
	 *  a saturation, not a model error, so the estimate is held. */
	void dob_post(double applied_thrust, bool saturated)
	{
		if (!_dob_en) { return; }

		_dob_prev_thrust_cmd = applied_thrust;
		_dob_hold = _dob_hold || saturated;
	}

	/** One observer step; returns d_hat, the acceleration the model fails to predict.
	 *
	 *  TIMING. accel_measured is what the vehicle did over the previous step, under the thrust commanded on
	 *  the previous tick and ending at the attitude it has now, so the model is built from exactly those.
	 *
	 *  MOTOR LAG. The plant lags rotor speed with time constant tau and T = b omega^2; linearized,
	 *  dT/dt = (T_d - T)/tau, so the thrust lags with the same tau. */
	V3 update_dob(const V3 &accel_measured, const Quat &quaternion, const V3 *velocity, double dt)
	{
		if (_dob_prev_thrust_cmd < 0.0) { return _dob_d; }   // first tick: nothing commanded yet

		if (_dob_thrust < 0.0) { _dob_thrust = _dob_prev_thrust_cmd; }

		const double alpha_motor = 1.0 - std::exp(-dt / _dob_mot_tau);
		_dob_thrust += alpha_motor * (_dob_prev_thrust_cmd - _dob_thrust);

		const M3 R = to_rotation_matrix(quaternion);
		const V3 body_z(R(0, 2), R(1, 2), R(2, 2));
		const double thrust_accel = _dob_thrust / _cap.mass;
		V3 a_model = body_z * thrust_accel - V3(0.0, 0.0, GRAVITY);

		// Drag only if the controller is told the velocity: then it is part of the model (and of the
		// (5.2) feed-forward). Without it, drag is simply one more thing the observer absorbs.
		if (velocity) { a_model = a_model - (*velocity) * _cap.linear_drag; }

		_dob_model = a_model;

		if (!_dob_filt_init) { _dob_meas_f = accel_measured; _dob_model_f = a_model; _dob_filt_init = true; }

		const double alpha_meas = dt / (_dob_meas_tau + dt);
		_dob_meas_f = _dob_meas_f + (accel_measured - _dob_meas_f) * alpha_meas;
		_dob_model_f = _dob_model_f + (a_model - _dob_model_f) * alpha_meas;

		if (!_dob_hold && !_dob_freeze) {
			const double alpha_d = dt / (_dob_tau + dt);
			const V3 error = _dob_meas_f - _dob_model_f;
			const double error_thrust = error.dot(body_z);
			_dob_d_thrust += alpha_d * (error_thrust - _dob_d_thrust);
			_dob_d_perp = _dob_d_perp + ((error - body_z * error_thrust) - _dob_d_perp) * alpha_d;
			_dob_d_perp = _dob_d_perp - body_z * _dob_d_perp.dot(body_z);   // b_z has moved

			const double limit = _dob_thrust_frac * thrust_accel;
			_dob_d_thrust = fmin(fmax(_dob_d_thrust, -limit), limit);
			const double n = _dob_d_perp.norm();

			if (n > _dob_max && n > 1e-12) { _dob_d_perp = _dob_d_perp * (_dob_max / n); }
		}

		_dob_d = body_z * _dob_d_thrust + _dob_d_perp;
		return _dob_d;
	}

	bool saturate_command(V3 &a_cmd) const
	{
		const double magnitude = a_cmd.norm();

		if (magnitude > _cap.max_lateral_accel() && magnitude > 1e-12) {
			a_cmd = a_cmd * (_cap.max_lateral_accel() / magnitude);
			return true;
		}

		return false;
	}

	bool prioritize_vertical(V3 &a_des) const
	{
		const double a_max = _max_thrust / _cap.mass;

		if (a_des.norm() <= a_max) { return false; }

		const double vertical = a_des(2);

		if (fabs(vertical) >= a_max) {
			a_des = V3(0.0, 0.0, fmin(fmax(vertical, -a_max), a_max));
			return true;
		}

		double hx = a_des(0), hy = a_des(1);
		const double allowed = std::sqrt(fmax(a_max * a_max - vertical * vertical, 0.0));
		const double h_norm = std::sqrt(hx * hx + hy * hy);

		if (h_norm > allowed && h_norm > 1e-12) {
			hx *= allowed / h_norm;
			hy *= allowed / h_norm;
		}

		a_des = V3(hx, hy, vertical);
		return true;
	}

	Quat desired_attitude(const V3 &a_des, double yaw_ref) const
	{
		const double norm = a_des.norm();

		if (norm < 1e-9) {
			return normalize(Quat{std::cos(0.5 * yaw_ref), 0.0, 0.0, std::sin(0.5 * yaw_ref)});
		}

		const V3 z_b = a_des / norm;
		const V3 x_c(std::cos(yaw_ref), std::sin(yaw_ref), 0.0);
		const V3 y_c(-std::sin(yaw_ref), std::cos(yaw_ref), 0.0);
		const V3 cross_x = cross(z_b, x_c);
		const V3 cross_y = cross(y_c, z_b);
		V3 x_b, y_b;

		if (cross_x.norm() >= cross_y.norm()) {
			y_b = cross_x / cross_x.norm();
			x_b = cross(y_b, z_b);

		} else {
			x_b = cross_y / cross_y.norm();
			y_b = cross(z_b, x_b);
		}

		M3 R;

		for (int i = 0; i < 3; i++) { R(i, 0) = x_b(i); R(i, 1) = y_b(i); R(i, 2) = z_b(i); }

		return from_rotation_matrix(R);
	}

	V3 attitude_error(const Quat &q_desired, const Quat &q_current) const
	{
		Quat q_err = normalize(multiply(conjugate(q_current), q_desired));

		if (q_err.w < 0.0) { q_err = Quat{-q_err.w, -q_err.x, -q_err.y, -q_err.z}; }

		return q_err.vec() * 2.0;
	}

	/** Replicates the paper plant's plus-frame allocation solve (quad_dynamics.motor_mixer). */
	bool mixer_saturates(double collective_thrust, const V3 &body_torque) const
	{
		const double l = _cap.arm_length, b = _cap.thrust_coefficient, d = _cap.torque_coefficient;
		matrix::SquareMatrix<double, 4> mix;
		const double m[4][4] = {{b, b, b, b}, {0.0, l * b, 0.0, -l * b}, {-l * b, 0.0, l * b, 0.0}, {d, -d, d, -d}};

		for (int i = 0; i < 4; i++) { for (int j = 0; j < 4; j++) { mix(i, j) = m[i][j]; } }

		matrix::SquareMatrix<double, 4> inv;

		if (!matrix::inv(mix, inv)) { return true; }

		matrix::Vector<double, 4> command;
		command(0) = collective_thrust;
		command(1) = body_torque(0);
		command(2) = body_torque(1);
		command(3) = body_torque(2);
		const matrix::Vector<double, 4> omega_sq = inv * command;
		const double w2max = _cap.max_motor_speed * _cap.max_motor_speed;

		for (int i = 0; i < 4; i++) { if (omega_sq(i) < 0.0 || omega_sq(i) > w2max) { return true; } }

		return false;
	}

	/** dt <= 0 disables the attitude-rate feed-forward, like dt=None in Python.
	 *  accel_measured (world frame, gravity removed) and dob_dt drive the (6) observer; dob_dt is the real
	 *  sample interval, kept separate from dt so that running without the rate feed-forward (fly -n) does
	 *  not also switch the observer off. */
	ControlOutput attitude_and_thrust(V3 commanded_acceleration, const Quat &quaternion, const V3 &body_rates,
					  double yaw_ref, const V3 *velocity, double dt,
					  const V3 *accel_measured = nullptr, double dob_dt = 0.0)
	{
		ControlOutput out;
		// 1. (5.1)
		V3 a_cmd = commanded_acceleration;
		out.saturated_accel = saturate_command(a_cmd);

		// 2. (5.2) gravity compensation + drag feed-forward
		V3 a_des = a_cmd + V3(0.0, 0.0, GRAVITY);

		if (velocity) { a_des = a_des + (*velocity) * _cap.linear_drag; }

		// 2b. (6) disturbance observer: remove what the model fails to predict. After the (5.1) envelope
		//     clip, like the drag feed-forward, because it corrects the MODEL and not the request; before
		//     (5.3), which stays the hard physical limit.
		const bool dob_active = _dob_en && accel_measured != nullptr && dob_dt > 0.0;

		if (dob_active) { a_des = a_des - update_dob(*accel_measured, quaternion, velocity, dob_dt); }

		// 3. (5.3)
		out.saturated_thrust = prioritize_vertical(a_des);

		// (5.9) upright regime (Sec. V-J): keep the thrust axis in the upper hemisphere by
		// zeroing only a downward vertical demand, before the frame (5.4) is built. (5.7a) needs
		// no change: it projects onto the actual body z and clips at 0.
		if (_cap.upright && a_des(2) < 0.0) { a_des(2) = 0.0; }

		const Quat q_des = desired_attitude(a_des, yaw_ref);

		// (5.7a) project onto the ACTUAL body z
		const M3 R = to_rotation_matrix(quaternion);
		const V3 body_z_actual(R(0, 2), R(1, 2), R(2, 2));
		const double thrust = _cap.mass * a_des.dot(body_z_actual);
		out.collective_thrust = fmin(fmax(thrust, 0.0), _max_thrust);

		// (5.6) attitude-rate feed-forward
		V3 omega_desired{};

		if (dt > 0.0 && _have_prev_q_des) {
			Quat dq = multiply(conjugate(_prev_q_des), q_des);

			if (dq.w < 0.0) { dq = Quat{-dq.w, -dq.x, -dq.y, -dq.z}; }

			const V3 omega_raw = dq.vec() * (2.0 / dt);
			const double tau_filter = 0.02;
			const double alpha = dt / (tau_filter + dt);
			_omega_des_filtered = _omega_des_filtered * (1.0 - alpha) + omega_raw * alpha;
			const M3 rotation_des_to_body = R.transpose() * to_rotation_matrix(q_des);
			omega_desired = rotation_des_to_body * _omega_des_filtered;
		}

		_prev_q_des = q_des;
		_have_prev_q_des = true;

		// (5.5), (5.7b)
		const V3 e_R = attitude_error(q_des, quaternion);
		const V3 rate_error = body_rates - omega_desired;
		const V3 kp = _cap.kp_att(), kd = _cap.kd_rate();
		out.body_torque = V3(kp(0) * e_R(0) - kd(0) * rate_error(0),
				     kp(1) * e_R(1) - kd(1) * rate_error(1),
				     kp(2) * e_R(2) - kd(2) * rate_error(2));

		const double c = a_des(2) / fmax(a_des.norm(), 1e-12);
		out.tilt_rad = std::acos(fmin(fmax(c, -1.0), 1.0));
		out.desired_quaternion = q_des;
		out.desired_acceleration = a_des;
		out.commanded_acceleration = a_des - V3(0.0, 0.0, GRAVITY);
		out.saturated_mixer = mixer_saturates(out.collective_thrust, out.body_torque);
		out.e_r = e_R;
		out.omega_desired = omega_desired;

		if (dob_active) {
			// Inputs to the NEXT tick's observer update (see update_dob, TIMING). The module overrides the
			// thrust through dob_post() with what it actually sent, and adds its own mixer saturation.
			_dob_prev_thrust_cmd = out.collective_thrust;
			_dob_hold = out.saturated_thrust || out.saturated_mixer;

		} else {
			// The observer did not run (no measurement, or disabled): T_hat and the measurement filters
			// would carry an arbitrarily old state into the tick it resumes on, and the difference against
			// a fresh a_meas would be read as a disturbance. Re-seed them instead. The learnt d_hat is
			// kept -- it is still the best estimate of the vehicle's model error.
			_dob_thrust = -1.0;
			_dob_filt_init = false;
		}

		out.dob_active = dob_active;
		out.accel_disturbance = _dob_d;
		out.accel_model = _dob_model;
		out.accel_disturbance_thrust = _dob_d_thrust;
		return out;
	}

	/** Reference-tracking mode (5.8). */
	ControlOutput compute(const V3 &position, const V3 &velocity, const Quat &quaternion, const V3 &body_rates,
			      const V3 &position_ref, const V3 &velocity_ref, const V3 &acceleration_ref,
			      double yaw_ref, double dt, V3 *a_cmd_raw = nullptr,
			      const V3 *accel_measured = nullptr, double dob_dt = 0.0)
	{
		const V3 pe = position_ref - position;
		const V3 ve = velocity_ref - velocity;
		const V3 a_cmd(acceleration_ref(0) + _cap.kp_pos(0) * pe(0) + _cap.kd_vel(0) * ve(0),
			       acceleration_ref(1) + _cap.kp_pos(1) * pe(1) + _cap.kd_vel(1) * ve(1),
			       acceleration_ref(2) + _cap.kp_pos(2) * pe(2) + _cap.kd_vel(2) * ve(2));

		if (a_cmd_raw) { *a_cmd_raw = a_cmd; }

		return attitude_and_thrust(a_cmd, quaternion, body_rates, yaw_ref, &velocity, dt, accel_measured, dob_dt);
	}

private:
	Capability _cap;
	double _max_thrust;
	Quat _prev_q_des{};
	bool _have_prev_q_des{false};
	V3 _omega_des_filtered{};

	// (6) disturbance observer; disabled until configure_dob() turns it on, so the controller is then
	// bit-identical to the paper's.
	bool _dob_en{false};
	double _dob_tau{0.2};           // [s] disturbance filter: larger = less noise, slower to learn
	double _dob_meas_tau{0.02};     // [s] applied identically to a_meas and a_model, so they stay in phase
	double _dob_max{0.3 * GRAVITY}; // [m/s^2] clip on the across-thrust part
	double _dob_thrust_frac{0.5};   // clip on the along-thrust part, fraction of T_hat/m
	double _dob_mot_tau{0.055};     // [s] motor time constant used for T_hat
	V3 _dob_d{}, _dob_d_perp{}, _dob_meas_f{}, _dob_model_f{}, _dob_model{};
	double _dob_d_thrust{0.0};
	double _dob_thrust{-1.0};           // T_hat, thrust after the motor lag; < 0: not initialised
	double _dob_prev_thrust_cmd{-1.0};  // thrust applied on the previous tick; < 0: nothing commanded yet
	bool _dob_filt_init{false};
	bool _dob_hold{false};              // previous tick saturated: the difference is not model error
	bool _dob_freeze{false};            // external gate (ground, rig, no measurement)
};

} // namespace capable
