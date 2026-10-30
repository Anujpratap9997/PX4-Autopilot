# capable_control: one sample per controller tick (logged at full rate).
# Frames follow the controller (MATH_Rishabh.pdf): world NWU (x north, y west, z up),
# body FLU (x forward, y left, z up = thrust axis). Quaternions are [w, x, y, z], body -> world.

uint64 timestamp          # [us] time of publication
uint64 timestamp_sample   # [us] gyro sample this tick was computed from

uint8 state               # module state, see STATE_*
uint8 STATE_IDLE = 0
uint8 STATE_ENGAGING = 1
uint8 STATE_FLY = 2
uint8 STATE_DISARMING = 3
uint8 STATE_DONE = 4
uint8 STATE_ABORTED = 5
uint8 STATE_HOLD = 6        # scenario complete, hovering in the air; accepts the next fly request

uint8 state_source        # state fed to the controller, see SOURCE_*
uint8 SOURCE_EKF = 0          # PX4's estimator (vehicle_local_position / attitude / angular_velocity)
uint8 SOURCE_GROUND_TRUTH = 1 # SIH ground truth (perfect state feedback, as in the paper's simulation)

uint8 phase               # scenario phase, see PHASE_*
uint8 PHASE_GROUND_SPINUP = 0
uint8 PHASE_TAKEOFF = 1
uint8 PHASE_HOVER = 2
uint8 PHASE_LAND = 3
uint8 PHASE_TOUCHDOWN = 4
uint8 PHASE_HOLD_0 = 5
uint8 PHASE_HOLD_1 = 6
uint8 PHASE_HOLD_2 = 7
uint8 PHASE_HOLD_3 = 8
uint8 PHASE_MOVE_NORTH = 9
uint8 PHASE_MOVE_WEST = 10
uint8 PHASE_MOVE_DIAGONAL = 11
uint8 PHASE_HOLD_PRE = 12
uint8 PHASE_CIRCLE_SPINUP = 13
uint8 PHASE_CIRCLE_STEADY = 14
uint8 PHASE_CIRCLE_SPINDOWN = 15
uint8 PHASE_HOLD_POST = 16
uint8 PHASE_ACCELERATE = 17
uint8 PHASE_CRUISE = 18
uint8 PHASE_PURSUIT = 19       # compute_pursuit: acc_ref holds the commanded acceleration, no position loop
uint8 PHASE_BRAKE = 20
uint8 PHASE_RETURN = 21        # fly back over the take-off point before landing
uint8 PHASE_DIVE = 22          # Sec. V-J test: fixed acceleration command (acc_ref), no position loop
uint8 PHASE_TILT = 24          # rig test (fly tilt -F): held pure-pitch or pure-roll tilt, or the ramps to and from it
uint8 PHASE_SYSID = 23         # motor identification: pseudo-random torque injected on one axis at a time (torque_inj)
uint8 PHASE_HOLD_IDLE = 25     # STATE_HOLD between fly commands (after an RC switch-in, the end of a test or a hand-back)
uint8 PHASE_ACCEL = 26         # fly accel: a fixed acceleration command (acc_ref), no position loop

bool upright              # Sec. V-J regime: true = upright (5.9), false = inverting

bool pursuit              # true: this tick ran compute_pursuit (a_cmd = acc_ref), not reference tracking

float32 t_fly             # [s] time since the scenario started
float32 dt                # [s] controller dt (gyro sample interval); 0 when the rate feed-forward is off

float64[3] pos            # [m] state used by the controller (EKF), NWU
float64[3] vel            # [m/s]
float64[4] q              # attitude
float64[3] rates          # [rad/s] body rates, FLU

float64[3] pos_ref        # reference trajectory
float64[3] vel_ref
float64[3] acc_ref
float64 yaw_ref           # [rad] NWU

float64[3] acc_cmd_raw    # [m/s^2] (5.8) before the (5.1) clip
float64[3] acc_cmd        # after (5.1)-(5.3), excluding gravity compensation
float64[3] acc_des        # (5.2)-(5.3), including gravity compensation and drag feed-forward
float64[4] q_des          # (5.4)
float64[3] e_r            # (5.5) attitude error proxy, body
float64[3] omega_ff       # (5.6b) filtered reference rate, desired frame
float64[3] omega_des      # (5.6c) feed-forward rate in the body frame
float32 att_err           # [rad] true rotation angle between q and q_des
float32 tilt              # [rad] actual tilt from vertical
float32 tilt_cmd          # [rad] commanded tilt

float64 thrust            # [N]   (5.7a)
float64[3] torque         # [N m] (5.7b), FLU
float64 thrust_ach        # [N]   produced by the clipped motor commands
float64[3] torque_ach     # [N m] produced by the clipped motor commands, FLU
float64[3] torque_inj     # [N m] sysid excitation added to torque_cmd before the mixer (0 otherwise), FLU
float64[3] torque_cmd     # [N m] torque sent to the mixer after the attitude compensation (CC_ATT_MODE), FLU
uint8 att_mode            # CC_ATT_MODE: 0 paper (5.7b), 1 motor-lag lead, 2 INDI, 3 both
bool virtual_translation  # fly -V: pos/vel are the virtual point mass driven by the real attitude (Stage B rig test)
bool ref_only             # fly -F: (5.8) bypassed, a_cmd = acc_ref and the drag feed-forward uses vel_ref (Stage A rig test)
float32 output_scale      # rig soft start: wrench scale 0..1 (1 outside rig mode)
bool thrust_capped        # rig mode: collective limited to CC_RIG_THR x max thrust
float32[4] u              # motor commands sent (actuator_motors), 0..1
float32[4] u_raw          # unclipped mixer solution

bool sat_accel            # (5.1) lateral command clipped
bool sat_thrust           # (5.3) thrust budget hit
bool sat_mixer            # a motor command was clipped (this vehicle's mixer)
bool sat_mixer_paper      # the controller's own prediction for the paper's plus-frame plant

# (6) acceleration disturbance observer (CC_DOB_EN). acc_meas is the measurement it is fed; comparing it
# with acc_des is the closed-loop version of the acc_cmd vs achieved check that ulog_accel_check.py does.
bool dob                  # the observer ran this tick (a_des had dob_d removed)
bool dob_frozen           # the estimate was held: on the ground, saturated, or no usable measurement
float64[3] acc_meas       # [m/s^2] measured acceleration fed to the observer, NWU, gravity removed
float64[3] dob_model      # [m/s^2] a_model: what the model says the vehicle should be doing
float64[3] dob_d          # [m/s^2] d_hat, subtracted from acc_des
float64 dob_d_thrust      # [m/s^2] along-thrust part of d_hat (scalar on b_z)

uint32 compute_us         # [us] time spent in the controller + mixer this tick

uint8 ORB_QUEUE_LENGTH = 8  # so the logger never drops a tick
