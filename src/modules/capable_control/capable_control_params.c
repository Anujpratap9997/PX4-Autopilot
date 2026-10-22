/**
 * @file capable_control_params.c
 * Vehicle model and gains for capable_control. Defaults = MATH_Rishabh.pdf Table II chaser.
 * Read once when a flight is requested (`capable_control fly ...`).
 * Rotor geometry comes from CA_ROTOR_COUNT / CA_ROTORi_PX / PY / KM, which must be in physical
 * units (metres, moment/thrust ratio in metres) in the airframe.
 */

/**
 * Vehicle mass
 * @unit kg
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_MASS, 3.0f);

/**
 * Inertia about body x
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_IXX, 0.025f);

/**
 * Inertia about body y
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_IYY, 0.025f);

/**
 * Inertia about body z
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_IZZ, 0.046f);

/**
 * Lateral acceleration limit (5.1)
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_ACC_MAX_G, 4.0f);

/**
 * Thrust clip, as thrust-to-weight ratio (5.3, 5.7a)
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_TW_MAX, 5.5f);

/**
 * Speed ceiling (admissibility only)
 * @unit m/s
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_V_MAX, 50.0f);

/**
 * Roll/pitch natural frequency (4.4)
 * @unit rad/s
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_ATT_WN, 14.0f);

/**
 * Roll/pitch damping ratio (4.4)
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_ATT_ZETA, 0.7f);

/**
 * Yaw natural frequency (4.4)
 * @unit rad/s
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_YAW_WN, 6.0f);

/**
 * Yaw damping ratio (4.4)
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_YAW_ZETA, 0.7f);

/**
 * Linear drag k_D used for feed-forward (5.2)
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_DRAG, 0.18f);

/**
 * Position gain kp_pos (5.8), all axes. Not from the paper excerpt.
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_POS_KP, 4.0f);

/**
 * Velocity gain kd_vel (5.8), all axes. Not from the paper excerpt.
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_VEL_KD, 3.6f);

/**
 * Maximum thrust of one rotor
 * @unit N
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_ROT_FMAX, 44.13f);

/**
 * Thrust curve of the motor command
 *
 * 0: thrust linear in the command (SIH). 1: thrust proportional to command squared
 * (command = rotor speed fraction, Gazebo / real ESCs).
 * @value 0 Linear
 * @value 1 Quadratic
 * @group Capable Control
 */
PARAM_DEFINE_INT32(CC_THR_MODEL, 0);

/**
 * Ground-truth source for the -t option
 *
 * 0: vehicle_*_groundtruth topics (SIH, exact at the sim rate).
 * 1: vehicle_visual_odometry from the Gazebo model's OdometryPublisher (gz_bridge's own
 * groundtruth is 50 Hz with Euler-difference rates, unusable for feedback).
 * @value 0 Groundtruth topics
 * @value 1 Visual odometry
 * @group Capable Control
 */
PARAM_DEFINE_INT32(CC_TRUTH_SRC, 0);

/**
 * Attitude regime (Sec. V-J)
 *
 * 0: inverting, thrust axis anywhere on the sphere (the shared controller, unmodified).
 * 1: upright, (5.9) projects a_des onto the upper half-space before (5.4), so the vehicle
 * never inverts and a downward break is capped at free fall.
 * @value 0 Inverting
 * @value 1 Upright
 * @group Capable Control
 */
PARAM_DEFINE_INT32(CC_UPRIGHT, 0);

/**
 * Attitude torque law (motor-lag study)
 *
 * 0: the paper's (5.7b), unmodified. 1: (5.7b) followed by a motor-lag lead
 * (T_m s + 1)/(T_c s + 1). 2: incremental (INDI) form of (5.7b): tau = tau_hat + J (alpha_des - wdot_hat),
 * alpha_des = (5.7b)/J, tau_hat = motor model (T_m) of the achieved torque, both filtered by T_c.
 * 3: mode 2 followed by the mode-1 lead.
 * @value 0 Paper
 * @value 1 Lead
 * @value 2 INDI
 * @value 3 INDI + lead
 * @group Capable Control
 */
PARAM_DEFINE_INT32(CC_ATT_MODE, 0);

/**
 * Motor time constant used by CC_ATT_MODE 1/2
 *
 * @unit s
 * @min 0.001
 * @max 0.5
 * @decimal 3
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_MOT_TAU, 0.055f);

/**
 * Compensation filter time constant (lead pole for mode 1, measurement filter for mode 2)
 *
 * @unit s
 * @min 0.001
 * @max 0.2
 * @decimal 3
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_COMP_TAU, 0.015f);

/**
 * Rig mode (static 3-DOF test rig)
 *
 * Enables the CC_RIG_* safety limits: collective cap, tilt and rate cutoffs, soft start.
 * 0 for simulation and free flight.
 * @boolean
 * @group Capable Control
 */
PARAM_DEFINE_INT32(CC_RIG_EN, 0);

/**
 * Rig: collective thrust cap, fraction of the maximum total thrust
 *
 * On the rig thrust does not move the vehicle; only differential thrust matters. The cap keeps
 * the bearing load and the energy in the props low.
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_RIG_THR, 0.35f);

/**
 * Rig: tilt cutoff
 *
 * Motors off and disarm when the tilt exceeds this angle (set it inside the rig's mechanical range).
 * @unit deg
 * @min 5
 * @max 180
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_RIG_TILT, 60.0f);

/**
 * Rig: body-rate cutoff
 *
 * Motors off and disarm when any body rate exceeds this value.
 * @unit deg/s
 * @min 30
 * @max 2000
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_RIG_RATE, 360.0f);

/**
 * Rig: soft-start time
 *
 * Thrust and torque are scaled from 0 to 1 over this time after the mode engages.
 * @unit s
 * @min 0
 * @max 10
 * @decimal 1
 * @group Capable Control
 */
PARAM_DEFINE_FLOAT(CC_RIG_RAMP, 2.0f);

/**
 * Rig: yaw axis controlled
 *
 * 0 when the rig's yaw axis is locked (stoppers): the module then commands no yaw torque, so the
 * whole motor range is left for roll and pitch (stock PX4 equivalent: MC_YAWRATE_K 0). Rig mode only.
 * @boolean
 * @group Capable Control
 */
PARAM_DEFINE_INT32(CC_RIG_YAW, 1);

/**
 * Start capable_control at boot
 *
 * 1: the module starts with the multicopter apps and registers the Capable Control mode (External Mode 1,
 * COM_FLTMODEx 100), so it can be put on the RC flight mode switch. 0: start it by hand (capable_control start).
 *
 * @boolean
 * @reboot_required true
 * @group Capable Control
 */
PARAM_DEFINE_INT32(CC_AUTOSTART, 0);
