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
		p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z,
		p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
		p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x,
		p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w};
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
};

class CapableFlightController
{
public:
	explicit CapableFlightController(const Capability &cap) : _cap(cap)
	{
		_max_thrust = cap.max_thrust_to_weight * cap.mass * GRAVITY;
	}

	void reset() { _have_prev_q_des = false; _omega_des_filtered.zero(); }

	const V3 &omega_des_filtered() const { return _omega_des_filtered; }
	double max_thrust() const { return _max_thrust; }

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

	/** dt <= 0 disables the attitude-rate feed-forward, like dt=None in Python. */
	ControlOutput attitude_and_thrust(V3 commanded_acceleration, const Quat &quaternion, const V3 &body_rates,
					  double yaw_ref, const V3 *velocity, double dt)
	{
		ControlOutput out;
		// 1. (5.1)
		V3 a_cmd = commanded_acceleration;
		out.saturated_accel = saturate_command(a_cmd);

		// 2. (5.2) gravity compensation + drag feed-forward
		V3 a_des = a_cmd + V3(0.0, 0.0, GRAVITY);

		if (velocity) { a_des = a_des + (*velocity) * _cap.linear_drag; }

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
		return out;
	}

	/** Reference-tracking mode (5.8). */
	ControlOutput compute(const V3 &position, const V3 &velocity, const Quat &quaternion, const V3 &body_rates,
			      const V3 &position_ref, const V3 &velocity_ref, const V3 &acceleration_ref,
			      double yaw_ref, double dt, V3 *a_cmd_raw = nullptr)
	{
		const V3 pe = position_ref - position;
		const V3 ve = velocity_ref - velocity;
		const V3 a_cmd(acceleration_ref(0) + _cap.kp_pos(0) * pe(0) + _cap.kd_vel(0) * ve(0),
			       acceleration_ref(1) + _cap.kp_pos(1) * pe(1) + _cap.kd_vel(1) * ve(1),
			       acceleration_ref(2) + _cap.kp_pos(2) * pe(2) + _cap.kd_vel(2) * ve(2));

		if (a_cmd_raw) { *a_cmd_raw = a_cmd; }

		return attitude_and_thrust(a_cmd, quaternion, body_rates, yaw_ref, &velocity, dt);
	}

private:
	Capability _cap;
	double _max_thrust;
	Quat _prev_q_des{};
	bool _have_prev_q_des{false};
	V3 _omega_des_filtered{};
};

} // namespace capable
