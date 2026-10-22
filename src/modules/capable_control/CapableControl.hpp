/**
 * @file CapableControl.hpp
 *
 * Standalone PX4 flight mode running the high-authority geometric controller of
 * MATH_Rishabh.pdf (capable_flight_controller.py) from state estimate straight to motors.
 *
 * The module registers its own flight mode with the commander (same mechanism as
 * mc_nn_control) and configures it with every PX4 control flag off -- no position, attitude,
 * rate controller or control allocator runs while it is active. It publishes actuator_motors
 * directly, on every vehicle_angular_velocity update (250 Hz in SIH), in the same work queue
 * as PX4's own rate controller.
 */

#pragma once

#include "CapableFlightController.hpp"
#include "Scenarios.hpp"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/capable_control_status.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>

#include <px4_platform_common/atomic.h>

using namespace time_literals;

class CapableControl : public ModuleBase<CapableControl>, public ModuleParams, public px4::WorkItem
{
public:
	CapableControl();
	~CapableControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

private:
	void Run() override;

	// commander external-mode plumbing (as in mc_nn_control)
	void register_mode();
	void check_registration();
	void configure_mode();
	void reply_arming_check(int8_t request_id);
	void send_command(uint32_t command, float p1 = 0.f, float p2 = 0.f);

	void control_tick(const vehicle_angular_velocity_s &rates);
	void publish_motors(const float u[4]);

	// Vehicle model (CC_* params) and mixer from CA_ROTOR* geometry (physical units),
	// per-rotor max thrust CC_ROT_FMAX and thrust curve CC_THR_MODEL.
	bool load_vehicle();
	void mix(double thrust, const capable::V3 &torque_frd, float u[4], float u_raw[4], bool &saturated) const;
	void wrench(const float u[4], double &thrust, capable::V3 &torque_frd) const;

	// uORB
	uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _gt_attitude_sub{ORB_ID(vehicle_attitude_groundtruth)};
	uORB::Subscription _gt_local_position_sub{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Subscription _gt_angular_velocity_sub{ORB_ID(vehicle_angular_velocity_groundtruth)};
	uORB::Subscription _visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};
	uORB::Subscription _register_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};   // stock output at an RC switch-in

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<capable_control_status_s> _status_pub{ORB_ID(capable_control_status)};
	// PX4's own setpoint topics (PX4 frames), so Flight Review / PlotJuggler show setpoint vs actual
	uORB::Publication<vehicle_attitude_setpoint_s> _att_sp_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s> _rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _lpos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};
	orb_advert_t _mavlink_log_pub{nullptr};   // QGC messages
	uORB::Publication<register_ext_component_request_s> _register_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<vehicle_control_mode_s> _config_control_setpoints_pub{ORB_ID(config_control_setpoints)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	vehicle_attitude_s _attitude{};
	vehicle_local_position_s _local_position{};
	vehicle_attitude_s _gt_attitude{};
	vehicle_local_position_s _gt_local_position{};
	vehicle_angular_velocity_s _gt_angular_velocity{};
	vehicle_odometry_s _visual_odometry{};
	int32_t _truth_src{0};
	bool _upright{false};           // CC_UPRIGHT (Sec. V-J regime), latched at each fly request          // CC_TRUTH_SRC: 0 *_groundtruth topics, 1 vehicle_visual_odometry
	bool _use_ground_truth{false};

	/** State fed to the controller this tick, in the controller's z-up frames. */
	void read_state(const vehicle_angular_velocity_s &rates, capable::V3 &p, capable::V3 &v,
			capable::Quat &q, capable::V3 &w) const;
	bool _requested_ground_truth{false};

	// state the controller sees this tick (EKF or ground truth), PX4 frames
	const vehicle_attitude_s &att() const { return _use_ground_truth ? _gt_attitude : _attitude; }
	const vehicle_local_position_s &lpos() const { return _use_ground_truth ? _gt_local_position : _local_position; }
	vehicle_status_s _vehicle_status{};

	// mode registration
	static constexpr int REQUEST_ID = 0x3c;
	bool _sent_registration{false};
	int8_t _mode_id{-1};
	int8_t _arming_check_id{-1};
	hrt_abstime _last_config_publish{0};

	// flight
	capable::CapableFlightController _controller{capable::chaser()};
	capable::Scenario _scenario{};
	uint8_t _state{capable_control_status_s::STATE_IDLE};
	hrt_abstime _t_fly0{0};
	// end of the last test (complete or aborted), for fly's wait in the console
	char _end_msg[120] {};
	bool _end_ok{false};
	px4::atomic<int> _end_seq{0};
	void end_test(bool ok, const char *fmt, ...);
	void publish_setpoints(const capable::Quat &qd, const capable::V3 &w_des, double thrust, const capable::Ref *ref);
	int wait_for_test();
	hrt_abstime _last_sample{0};
	hrt_abstime _last_command{0};
	hrt_abstime _engage_start{0};
	capable::V3 _p0{};
	capable::V3 _home{};         // take-off point on the ground (first fly from the ground)
	capable::V3 _hold_p{};       // STATE_HOLD: hover point
	double _hold_yaw{0.0};
	capable::V3 _last_pursuit_acmd{};
	double _last_pursuit_yaw{0.0};

	// request from the shell (custom_command runs on another thread)
	px4::atomic_bool _fly_requested{false};
	capable::Scenario::Options _requested{};
	bool _rate_ff{true};

	// mixer
	bool _mixer_ok{false};
	float _f_max{0.f};

	// attitude compensation (CC_ATT_MODE), state persists across FLY / HOLD; reset on a ground start
	int32_t _att_mode{0};
	double _mot_tau{0.055}, _comp_tau{0.015};
	capable::V3 _J{};
	capable::V3 _lead_x{}, _m_hat{}, _tau_hat{}, _wdot_hat{}, _w_prev{}, _tau_ach_prev{};
	bool _comp_init{false};
	double _sysid_t0{-1.0};          // t_fly at the start of PHASE_SYSID
	double _sysid_scale{1.0};        // fly -A

	// rig safety (CC_RIG_*), latched at each fly request
	bool _rig{false};
	bool _rig_yaw{true};    // CC_RIG_YAW: false = yaw axis locked on the rig, no yaw torque
	double _rig_thr{0.35}, _rig_tilt{60.0}, _rig_rate{360.0}, _rig_ramp{2.0};
	hrt_abstime _engaged_at{0};
	double _engage_thrust{-1.0};   // [N] collective at an RC switch-in (rig: blended from it, torque not scaled); -1: soft start from 0
	bool _prev_in_mode_armed{false};
	const char *rig_violation(const capable::Quat &q, const capable::V3 &w) const;
	const char *state_stale(hrt_abstime now) const;
	void print_params() const;

	// Stage B (fly -V): virtual point mass, driven by the real attitude and the commanded thrust
	bool _virt_init{false};
	capable::V3 _vp{}, _vv{};
	double _v_thrust{0.0}, _v_ground{0.0}, _mass{3.0}, _kdrag{0.18};
	void virtual_init(const capable::V3 &p, const capable::V3 &v, double thrust, double ground)
	{
		_vp = p; _vv = v; _v_thrust = thrust; _v_ground = ground; _virt_init = true;
	}
	void virtual_step(const capable::Quat &q, double thrust, double dt);
	void rig_limit(double &thrust, capable::V3 &torque, double now_s, float &scale, bool &capped) const;
	capable::V3 sysid_injection(double t);
	capable::V3 compensate(const capable::V3 &tau_paper, const capable::V3 &w, double dt);
	void compensate_post(const capable::V3 &tau_ach) { _tau_ach_prev = tau_ach; }
	int32_t _thr_model{0};          // 0 linear (SIH), 1 quadratic (gz / ESC)
	float _rotor_px[4] {}, _rotor_py[4] {}, _rotor_km[4] {};
	matrix::SquareMatrix<double, 4> _A{}, _A_inv{};   // wrench = A * f, f = per-rotor thrust / CC_ROT_FMAX

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
