/**
 * @file Scenarios.hpp
 *
 * Port of px4_offboard/scenarios.py: reference trajectories in the controller's NWU frame.
 * Each scenario starts on the ground at the engage position and ends back on the ground.
 */

#pragma once

#include "CapableFlightController.hpp"

#include <uORB/topics/capable_control_status.h>

namespace capable
{

struct Ref {
	V3 p, v, a;
	double yaw{0.0};
	uint8_t phase{0};
};

inline void min_jerk(double tau, double T, double &s, double &ds, double &dds)
{
	const double x = fmin(fmax(tau / T, 0.0), 1.0);
	s = 10 * pow(x, 3) - 15 * pow(x, 4) + 6 * pow(x, 5);
	ds = (30 * x * x - 60 * pow(x, 3) + 30 * pow(x, 4)) / T;
	dds = (60 * x - 180 * x * x + 120 * pow(x, 3)) / (T * T);
}

inline double wrap_pi(double a) { return fmod(fmod(a + M_PI, 2 * M_PI) + 2 * M_PI, 2 * M_PI) - M_PI; }

struct Segment {
	enum Kind { MOVE, HOLD, CIRCLE, SPEEDRAMP, CRUISE, QUINTIC, ACCEL } kind{HOLD};
	// accel: acceleration aa0 -> aa1 (minimum jerk), velocity and position integrated from (p0, av0)
	V3 av0, aa0, aa1, end_v;
	// speed ramp / cruise (straight line along +x)
	double speed{0.0};
	// quintic brake: per-axis coefficients c[k](axis)
	V3 qc[6];
	double duration{0.0};
	uint8_t phase{0};
	// move / hold
	V3 p0, p1;
	double yaw0{0.0}, dyaw{0.0};
	// circle
	V3 center;
	double R{0.0}, th0{0.0}, w{0.0}, Tu{0.0}, Ts{0.0};
	bool velocity_yaw{true};
	double fixed_yaw{0.0};

	V3 end_p;
	double end_yaw{0.0};

	static Segment move(const V3 &a, const V3 &b, double ya, double yb, double T, uint8_t ph)
	{
		Segment s;
		s.kind = MOVE; s.p0 = a; s.p1 = b; s.yaw0 = ya; s.dyaw = wrap_pi(yb - ya); s.duration = T; s.phase = ph;
		s.end_p = b; s.end_yaw = ya + s.dyaw;
		return s;
	}

	static Segment hold(const V3 &p, double yaw, double T, uint8_t ph)
	{
		Segment s;
		s.kind = HOLD; s.p0 = p; s.yaw0 = yaw; s.duration = T; s.phase = ph;
		s.end_p = p; s.end_yaw = yaw;
		return s;
	}

	static Segment circle(const V3 &c, double R, double th0, double w, double Tu, double revs, bool vel_yaw,
			      double fixed_yaw)
	{
		Segment s;
		s.kind = CIRCLE; s.center = c; s.R = R; s.th0 = th0; s.w = w; s.Tu = Tu; s.Ts = revs * 2 * M_PI / w;
		s.duration = 2 * Tu + s.Ts; s.velocity_yaw = vel_yaw; s.fixed_yaw = fixed_yaw;
		s.phase = capable_control_status_s::PHASE_CIRCLE_STEADY;
		double th, thd, thdd; uint8_t ph;
		s.theta(s.duration, th, thd, thdd, ph);
		s.end_p = c + V3(cos(th), sin(th), 0.0) * R;
		s.end_yaw = s.yaw_of(th);
		return s;
	}

	static Segment speed_ramp(const V3 &p, double yaw, double v, double T, uint8_t ph)
	{
		Segment s;
		s.kind = SPEEDRAMP; s.p0 = p; s.yaw0 = yaw; s.speed = v; s.duration = T; s.phase = ph;
		s.end_p = p + V3(0.5 * v * T, 0.0, 0.0); s.end_yaw = yaw;
		return s;
	}

	static Segment cruise(const V3 &p, double yaw, double v, double T, uint8_t ph)
	{
		Segment s;
		s.kind = CRUISE; s.p0 = p; s.yaw0 = yaw; s.speed = v; s.duration = T; s.phase = ph;
		s.end_p = p + V3(v * T, 0.0, 0.0); s.end_yaw = yaw;
		return s;
	}

	/** Acceleration blended from a0 to a1 over T (minimum jerk), integrated from (p0, v0) (scenarios.Accel). */
	static Segment accel(const V3 &p0, const V3 &v0, const V3 &a0, const V3 &a1, double yaw, double T, uint8_t ph)
	{
		Segment s;
		s.kind = ACCEL; s.p0 = p0; s.av0 = v0; s.aa0 = a0; s.aa1 = a1; s.yaw0 = yaw; s.duration = T; s.phase = ph;
		const V3 d = a1 - a0;
		s.end_v = v0 + a0 * T + d * (0.5 * T);
		s.end_p = p0 + v0 * T + a0 * (0.5 * T * T) + d * (T * T / 7.0);
		s.end_yaw = yaw;
		return s;
	}
	/** Per-axis quintic from (p0, v0, a0) to rest at p1 after T (same as scenarios.Quintic). */
	static Segment quintic(const V3 &p0, const V3 &v0, const V3 &a0, const V3 &p1, double yaw, double T, uint8_t ph)
	{
		Segment s;
		s.kind = QUINTIC; s.yaw0 = yaw; s.duration = T; s.phase = ph; s.end_p = p1; s.end_yaw = yaw;
		// [T^3 T^4 T^5; 3T^2 4T^3 5T^4; 6T 12T^2 20T^3] [c3 c4 c5]' = rhs, solved in closed form
		matrix::SquareMatrix<double, 3> M;
		M(0, 0) = pow(T, 3); M(0, 1) = pow(T, 4); M(0, 2) = pow(T, 5);
		M(1, 0) = 3 * T * T; M(1, 1) = 4 * pow(T, 3); M(1, 2) = 5 * pow(T, 4);
		M(2, 0) = 6 * T; M(2, 1) = 12 * T * T; M(2, 2) = 20 * pow(T, 3);
		matrix::SquareMatrix<double, 3> Mi;
		matrix::inv(M, Mi);
		s.qc[0] = p0; s.qc[1] = v0; s.qc[2] = a0 * 0.5;

		for (int ax = 0; ax < 3; ax++) {
			const V3 rhs(p1(ax) - p0(ax) - v0(ax) * T - 0.5 * a0(ax) * T * T, -v0(ax) - a0(ax) * T, -a0(ax));
			const V3 c345 = Mi * rhs;
			s.qc[3](ax) = c345(0); s.qc[4](ax) = c345(1); s.qc[5](ax) = c345(2);
		}

		return s;
	}

	double yaw_of(double theta) const { return velocity_yaw ? theta + M_PI / 2 : fixed_yaw; }

	void theta(double tau, double &th, double &thd, double &thdd, uint8_t &ph) const
	{
		auto up_int = [](double s) { return 2.5 * pow(s, 4) - 3 * pow(s, 5) + pow(s, 6); };
		double b, db, ddb;

		if (tau < Tu) {
			min_jerk(tau, Tu, b, db, ddb);
			th = th0 + w * Tu * up_int(tau / Tu); thd = w * b; thdd = w * db;
			ph = capable_control_status_s::PHASE_CIRCLE_SPINUP;
			return;
		}

		const double th1 = th0 + 0.5 * w * Tu;

		if (tau < Tu + Ts) {
			th = th1 + w * (tau - Tu); thd = w; thdd = 0.0;
			ph = capable_control_status_s::PHASE_CIRCLE_STEADY;
			return;
		}

		const double td = fmin(tau - Tu - Ts, Tu);
		const double s = td / Tu;
		min_jerk(td, Tu, b, db, ddb);
		th = th1 + w * Ts + w * Tu * (s - up_int(s)); thd = w * (1 - b); thdd = -w * db;
		ph = capable_control_status_s::PHASE_CIRCLE_SPINDOWN;
	}

	Ref ref(double tau) const
	{
		Ref r;
		r.phase = phase;

		if (kind == MOVE) {
			double s, ds, dds;
			min_jerk(tau, duration, s, ds, dds);
			const V3 d = p1 - p0;
			r.p = p0 + d * s; r.v = d * ds; r.a = d * dds; r.yaw = yaw0 + s * dyaw;

		} else if (kind == HOLD) {
			r.p = p0; r.v.zero(); r.a.zero(); r.yaw = yaw0;

		} else if (kind == SPEEDRAMP) {
			const double x = fmin(fmax(tau / duration, 0.0), 1.0);
			double b, db, ddb;
			min_jerk(tau, duration, b, db, ddb);
			const double dist = speed * duration * (2.5 * pow(x, 4) - 3 * pow(x, 5) + pow(x, 6));
			r.p = p0 + V3(dist, 0.0, 0.0); r.v = V3(speed * b, 0.0, 0.0); r.a = V3(speed * db, 0.0, 0.0); r.yaw = yaw0;

		} else if (kind == CRUISE) {
			r.p = p0 + V3(speed * tau, 0.0, 0.0); r.v = V3(speed, 0.0, 0.0); r.a.zero(); r.yaw = yaw0;

		} else if (kind == ACCEL) {
			const double x = fmin(fmax(tau / duration, 0.0), 1.0), t = x * duration;
			double b, db, ddb;
			min_jerk(t, duration, b, db, ddb);
			const V3 d = aa1 - aa0;
			const double ui = 2.5 * pow(x, 4) - 3 * pow(x, 5) + pow(x, 6);
			const double ui2 = 0.5 * pow(x, 5) - 0.5 * pow(x, 6) + pow(x, 7) / 7.0;
			r.p = p0 + av0 * t + aa0 * (0.5 * t * t) + d * (duration * duration * ui2);
			r.v = av0 + aa0 * t + d * (duration * ui);
			r.a = aa0 + d * b;
			r.yaw = yaw0;

		} else if (kind == QUINTIC) {
			const double s = fmin(fmax(tau, 0.0), duration);
			r.p = qc[0] + qc[1] * s + qc[2] * (s * s) + qc[3] * pow(s, 3) + qc[4] * pow(s, 4) + qc[5] * pow(s, 5);
			r.v = qc[1] + qc[2] * (2 * s) + qc[3] * (3 * s * s) + qc[4] * (4 * pow(s, 3)) + qc[5] * (5 * pow(s, 4));
			r.a = qc[2] * 2.0 + qc[3] * (6 * s) + qc[4] * (12 * s * s) + qc[5] * (20 * pow(s, 3));
			r.yaw = yaw0;

		} else {
			double th, thd, thdd;
			theta(tau, th, thd, thdd, r.phase);
			const V3 radial(cos(th), sin(th), 0.0), tangent(-sin(th), cos(th), 0.0);
			r.p = center + radial * R;
			r.v = tangent * (R * thd);
			r.a = tangent * (R * thdd) - radial * (R * thd * thd);
			r.yaw = yaw_of(th);
		}

		return r;
	}
};

class Scenario
{
public:
	static constexpr int MAX_SEGMENTS = 32;

	enum Type { HOVER, TRANSLATE, CIRCLE, PURSUIT, DIVE, SYSID, TILT, ACCEL };

	struct Options {
		Type type{HOVER};
		double g_level{1.0};
		double speed{20.0};
		double revs{3.0};
		bool velocity_yaw{true};
		double altitude{50.0};   // circle / pursuit (clear of Baylands trees)
		double turn_time{10.0};  // pursuit: turn duration; dive: dive duration; tilt: ramp time
		double pursuit_ramp{0.0};  // pursuit: smoothstep entry of the turn command [s]; 0 = step
		double down_g{2.0};      // dive: commanded downward acceleration [g]
		double lat_g{0.0};       // dive: commanded lateral (north) acceleration [g]
		// accel: the commanded acceleration [m/s^2], NWU (x north, y west, z up), gravity EXCLUDED, so
		// (0, 0, 0) is "keep the current velocity". Held for turn_time, then braked. The same quantity
		// and the same units as the Python bench's --accel.
		V3 accel_cmd{};
		bool land{false};        // end with return + landing; default: brake and hold in the air
		bool ref_only{false};    // fly -F: attitude-only, (5.8) bypassed (rig Stage A)
		bool virt{false};        // fly -V: virtual translation driven by the real attitude (rig Stage B)
	};

	/** p0: take-off point on the ground. start: current position when already airborne (the
	 *  ground spin-up and take-off are replaced by a transit to p0 + altitude), else nullptr. */
	void build(const Options &o, const V3 &p0, double yaw0, const V3 *start = nullptr)
	{
		_n = 0;
		_opt = o;
		_air_start = start;
		using S = capable_control_status_s;

		if (o.type == HOVER) {
			takeoff(p0, yaw0, 5.0, 4.0, yaw0);
			const V3 top = last().end_p;
			add(Segment::hold(top, last().end_yaw, 15.0, S::PHASE_HOVER));

		} else if (o.type == SYSID) {
			// Motor identification: hover at 10 m; the module injects a pseudo-random torque during
			// PHASE_SYSID, 4 s on roll, then pitch, then yaw.
			takeoff(p0, yaw0, 10.0, 4.0, yaw0);
			const V3 top = last().end_p;
			add(Segment::hold(top, last().end_yaw, 2.0, S::PHASE_HOLD_PRE));
			add(Segment::hold(top, last().end_yaw, 12.0, S::PHASE_SYSID));

		} else if (o.type == TILT) {
			// Rig test (attitude only, -F): tilt to g_level g forward, back, right, left with the heading fixed:
			// each ramped over turn_time (minimum jerk), held 3 s, ramped back, then 3 s level. Pure pitch or pure
			// roll, so the two q_des constructions agree (no heading jump). Forward/back and right/left cancel:
			// the reference ends at rest.
			takeoff(p0, yaw0, 10.0, 4.0, yaw0);
			const V3 top = last().end_p;
			const double y = last().end_yaw;
			add(Segment::hold(top, y, 3.0, S::PHASE_HOLD_PRE));
			const double A = o.g_level * GRAVITY, T = fmax(o.turn_time, 0.5), H = 3.0, L = 3.0;
			const V3 fwd(cos(y), sin(y), 0.0), right(sin(y), -cos(y), 0.0);
			V3 z; z.zero();
			const V3 dirs[4] = {fwd, -fwd, right, -right};
			const uint8_t level[3] = {S::PHASE_HOLD_0, S::PHASE_HOLD_1, S::PHASE_HOLD_2};
			V3 p = top, v; v.zero();

			for (int i = 0; i < 4; i++) {
				const V3 a = dirs[i] * A;
				add(Segment::accel(p, v, z, a, y, T, S::PHASE_TILT)); p = last().end_p; v = last().end_v;
				add(Segment::accel(p, v, a, a, y, H, S::PHASE_TILT)); p = last().end_p; v = last().end_v;
				add(Segment::accel(p, v, a, z, y, T, S::PHASE_TILT)); p = last().end_p; v = last().end_v;

				if (i < 3) { add(Segment::accel(p, v, z, z, y, L, level[i])); p = last().end_p; v = last().end_v; }
			}

		} else if (o.type == TRANSLATE) {
			takeoff(p0, yaw0, 10.0, 4.0, yaw0);
			const V3 top = last().end_p;
			const double y = last().end_yaw;
			const V3 a = top + V3(20.0, 0.0, 0.0), b = top + V3(20.0, 20.0, 0.0);
			add(Segment::hold(top, y, 3.0, S::PHASE_HOLD_0));
			add(Segment::move(top, a, y, y, 4.0, S::PHASE_MOVE_NORTH));
			add(Segment::hold(a, y, 4.0, S::PHASE_HOLD_1));
			add(Segment::move(a, b, y, y, 4.0, S::PHASE_MOVE_WEST));
			add(Segment::hold(b, y, 4.0, S::PHASE_HOLD_2));
			add(Segment::move(b, top, y, y, 4.0, S::PHASE_MOVE_DIAGONAL));
			add(Segment::hold(top, y, 4.0, S::PHASE_HOLD_3));

		} else if (o.type == ACCEL) {
			// Closed-loop acceleration test: hold at altitude, then the module flies the acceleration
			// phase itself (a fixed a_cmd, no position loop) and calls append_brake_and_land().
			// Same structure as DIVE, with an arbitrary 3D command instead of down_g / lat_g.
			takeoff(p0, yaw0, o.altitude, 8.0, yaw0);
			add(Segment::hold(last().end_p, last().end_yaw, 3.0, S::PHASE_HOLD_PRE));
			finalize();
			_pursuit_start = _duration;
			_duration += o.turn_time;
			return;

		} else if (o.type == DIVE) {
			// Hover at altitude; the module then flies the dive phase itself (fixed a_cmd) and
			// calls append_brake_and_land().
			takeoff(p0, yaw0, o.altitude, 8.0, yaw0);
			add(Segment::hold(last().end_p, last().end_yaw, 3.0, S::PHASE_HOLD_PRE));
			finalize();
			_pursuit_start = _duration;
			_duration += o.turn_time;
			return;

		} else if (o.type == PURSUIT) {
			// Run-up only; the module flies the pursuit phase itself and then calls append_brake_and_land().
			takeoff(p0, yaw0, o.altitude, 6.0, yaw0);
			const V3 top = last().end_p;
			const double accel_time = fmax(6.0, 1.875 * o.speed / (0.5 * GRAVITY));
			add(Segment::hold(top, last().end_yaw, 2.0, S::PHASE_HOLD_PRE));
			add(Segment::speed_ramp(top, yaw0, o.speed, accel_time, S::PHASE_ACCELERATE));
			add(Segment::cruise(last().end_p, yaw0, o.speed, 1.0, S::PHASE_CRUISE));
			finalize();
			_pursuit_start = _duration;
			_duration += o.turn_time;      // pursuit phase: no reference
			return;

		} else {
			const double a_c = o.g_level * GRAVITY;
			_radius = o.speed * o.speed / a_c;
			_rate = a_c / o.speed;
			const double spin = fmax(8.0, 1.875 * o.speed / (0.6 * GRAVITY));
			const double circle_yaw = o.velocity_yaw ? M_PI / 2 : yaw0;
			takeoff(p0, yaw0, o.altitude, 6.0, circle_yaw);
			const V3 top = last().end_p;
			_center = top - V3(_radius, 0.0, 0.0);
			add(Segment::hold(top, last().end_yaw, 3.0, S::PHASE_HOLD_PRE));
			add(Segment::circle(_center, _radius, 0.0, _rate, spin, o.revs, o.velocity_yaw, yaw0));
			add(Segment::hold(last().end_p, last().end_yaw, 3.0, S::PHASE_HOLD_POST));
		}

		land(p0);
		finalize();
	}

	/** Pursuit: at time t (end of the pursuit phase), brake from the current state with a quintic
	 *  and land. Peak braking 1.5 v/T is held to 0.5 g; the lateral acceleration a decays to 0. */
	void append_brake_and_land(double t, const V3 &p, const V3 &v, const V3 &a, double yaw, const V3 &p0,
				   double brake_g = 0.5)
	{
		using S = capable_control_status_s;
		const double T = fmax(3.0, 1.5 * v.norm() / (brake_g * GRAVITY));
		const V3 p1 = p + v * (0.5 * T);
		const int first = _n;
		add(Segment::quintic(p, v, a, p1, yaw, T, S::PHASE_BRAKE));
		land(p0);
		double s = t;

		for (int i = first; i < _n; i++) { _start[i] = s; s += _seg[i].duration; }

		_duration = s;
		_pursuit_start = -1.0;   // pursuit over: ref() is valid again from t onwards
	}

	/** True while t is inside the pursuit phase (the module computes a_cmd from the state). */
	bool in_pursuit(double t) const { return _pursuit_start >= 0.0 && t >= _pursuit_start; }
	double pursuit_start() const { return _pursuit_start; }
	double pursuit_end() const { return _pursuit_start + _opt.turn_time; }
	const Options &options() const { return _opt; }

	Ref ref(double t) const
	{
		int i = 0;

		while (i + 1 < _n && t >= _start[i + 1]) { i++; }

		return _seg[i].ref(t - _start[i]);
	}

	double duration() const { return _duration; }
	double radius() const { return _radius; }
	double rate() const { return _rate; }
	const V3 &center() const { return _center; }

private:
	void finalize()
	{
		_duration = 0.0;

		for (int i = 0; i < _n; i++) { _start[i] = _duration; _duration += _seg[i].duration; }
	}

	void add(const Segment &s) { if (_n < MAX_SEGMENTS) { _seg[_n++] = s; } }

	/** Fly back over the take-off point (same altitude) before descending (scenarios.py: "return"). */
	void return_home(const V3 &p0)
	{
		const V3 last_p = last().end_p;
		const V3 home(p0(0), p0(1), last_p(2));
		const double dist = (home - last_p).norm();

		if (dist > 0.5) {
			add(Segment::move(last_p, home, last().end_yaw, last().end_yaw, move_time(dist, 10.0),
					  capable_control_status_s::PHASE_RETURN));
		}
	}
	const Segment &last() const { return _seg[_n - 1]; }

	void takeoff(const V3 &p0, double yaw0, double altitude, double takeoff_time, double yaw_top)
	{
		using S = capable_control_status_s;
		const V3 top = p0 + V3(0.0, 0.0, altitude);

		if (_air_start) {
			// Already airborne: transit to top, horizontally at the higher of the two altitudes
			// (never descend diagonally into scenery), then a 2 s settle.
			const V3 s = *_air_start;
			const V3 corner(s(2) >= top(2) ? top(0) : s(0), s(2) >= top(2) ? top(1) : s(1), s(2) >= top(2) ? s(2) : top(2));
			add(Segment::move(s, corner, yaw0, yaw0, move_time((corner - s).norm(), 10.0), S::PHASE_TAKEOFF));
			add(Segment::move(corner, top, yaw0, yaw_top, move_time((top - corner).norm(), 10.0), S::PHASE_TAKEOFF));
			add(Segment::hold(top, yaw_top, 2.0, S::PHASE_TAKEOFF));
			return;
		}

		add(Segment::hold(p0, yaw0, 1.0, S::PHASE_GROUND_SPINUP));
		add(Segment::move(p0, top, yaw0, yaw_top, takeoff_time, S::PHASE_TAKEOFF));
	}

	void land(const V3 &p0)
	{
		if (!_opt.land) {
			add(Segment::hold(last().end_p, last().end_yaw, 2.0, capable_control_status_s::PHASE_HOLD_POST));
			return;
		}

		return_home(p0);
		descend(last().end_p, p0, last().end_yaw);
	}

	/** Quintic move duration over dist: peak speed 1.875 d/T <= v_max, peak accel 5.77 d/T^2 <= a_max. */
	static double move_time(double dist, double v_max, double a_max = 0.5 * GRAVITY)
	{
		return fmax(fmax(1.875 * dist / v_max, sqrt(5.7735 * dist / a_max)), 2.0);
	}

	/** Land from top: fast to 3 m above p0 (<= 10 m/s, 0.5 g), then the last 3 m at <= 1.5 m/s. */
	void descend(V3 top, const V3 &p0, double yaw)
	{
		using S = capable_control_status_s;
		const V3 ground(top(0), top(1), p0(2));

		if (top(2) - p0(2) > 3.5) {
			const V3 low = ground + V3(0.0, 0.0, 3.0);
			add(Segment::move(top, low, yaw, yaw, move_time(top(2) - low(2), 10.0), S::PHASE_LAND));
			top = low;
		}

		add(Segment::move(top, ground, yaw, yaw, fmax(1.875 * (top(2) - p0(2)) / 1.5, 2.0), S::PHASE_LAND));
		add(Segment::hold(ground - V3(0.0, 0.0, 0.1), yaw, 1.0, S::PHASE_TOUCHDOWN));
	}

	const V3 *_air_start{nullptr};   // valid during build() only
	Segment _seg[MAX_SEGMENTS];
	double _start[MAX_SEGMENTS] {};
	int _n{0};
	double _duration{0.0};
	Options _opt{};
	double _radius{0.0}, _rate{0.0};
	V3 _center{};
	double _pursuit_start{-1.0};
};

} // namespace capable
