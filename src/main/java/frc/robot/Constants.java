package frc.robot;

import frc.robot.util.PIDConstants;

public final class Constants {
    public static enum Robot {
        PRACTICE, COMPETITION;
    }

    public static final Robot ROBOT_TYPE = Robot.PRACTICE;

    //teleoperated control definitions
    public final static int DRIVER_JOYSTICK_ID = 0;
    public final static double DRIVER_JOYSTICK_DEADZONE = 0.13;

    public final static int OPERATOR_JOYSTICK_ID = 1;
    public final static double OPERATOR_JOYSTICK_DEADZONE = 0.1;

    public final static int BUTTONBOARD_JOYSTICK_ID = 2;

    //pneumatic definitions
    public final static int COMPRESSOR_PCM_ID = 30;

    public final static int ELEVATOR_BRAKE_PCM_ID = 30;
    public final static int ELEVATOR_BRAKE_ID = 0;

    public final static int ARM_BRAKE_PCM_ID = 30;
    public final static int ARM_BRAKE_ID = ROBOT_TYPE == Robot.COMPETITION ? 5 : 1;

    public final static int INTAKE_JAY_PCM_ID = 30;
    public final static int INTAKE_JAY_ID = ROBOT_TYPE == Robot.COMPETITION ? 7 : 4;

    public final static int INTAKE_FINGER_PCM_ID = 30;
    public final static int INTAKE_FINGER_FORWARD_ID = 6;
    public final static int INTAKE_FINGER_REVERSE_ID = ROBOT_TYPE == Robot.COMPETITION ? 4 : 7;

    public final static int CLIMBER_FORK_PCM_ID = 30;
    public final static int CLIMBER_FORK_ID = 2;

    //motor controller definitions

    // Victor SPX's
    // These are PWM ID's, not CAN ID's
    public final static int CLIMBER_WINCH_LEFT_ID = 0;
    public final static boolean CLIMBER_WINCH_LEFT_INVERTED = false;

    public final static int CLIMBER_WINCH_RIGHT_ID = 1;
    public final static boolean CLIMBER_WINCH_RIGHT_INVERTED = false;

    //Spark MAX's
    public final static int DRIVEBASE_FRONT_LEFT_ID = 4;
    public final static boolean DRIVEBASE_FRONT_LEFT_INVERTED = true;

    public final static int DRIVEBASE_FRONT_RIGHT_ID = 7;
    public final static boolean DRIVEBASE_FRONT_RIGHT_INVERTED = false;

    // NOTE: The back motor follows the front so keep that in mind when inverting

    public final static int DRIVEBASE_BACK_LEFT_ID = 5;
    public final static boolean DRIVEBASE_BACK_LEFT_INVERTED = true;
    
    public final static int DRIVEBASE_BACK_RIGHT_ID = 6;
    public final static boolean DRIVEBASE_BACK_RIGHT_INVERTED = false;

    public final static int CLIMBER_LEFT_ID = 8;
    public final static boolean CLIMBER_LEFT_INVERTED = false;

    public final static int CLIMBER_RIGHT_ID = 9;
    public final static boolean CLIMBER_RIGHT_INVERTED = true;

    public final static int ARM_PIVOT_ID = 10;
    public final static boolean ARM_PIVOT_INVERTED = true;

    // Talon SRX's
    public final static int INTAKE_ROLLER_ID = 1;
    public final static boolean INTAKE_ROLLER_INVERTED = true;

    public final static int ELEVATOR_BOTTOM_ID = 2;
    public final static boolean ELEVATOR_BOTTOM_INVERTED = true;

    public final static int ELEVATOR_TOP_ID = 3;
    public final static boolean ELEVATOR_TOP_INVERTED = false;

    // sensors

    // analog inputs
    public final static int ARM_ENCODER_PORT = 3;

    public final static double ARM_ENCODER_CALIBRATION_OFFSET = ROBOT_TYPE == Robot.COMPETITION ? 100 : 234;
    // to calculate this, move the elevator to the max operational height and record the number
    public final static double ELEVATOR_ENCODER_CALIBRATION = 43000;

    // digital inputs

    public final static int ELEVATOR_CARRIAGE_UP_PORT = 4;
    public final static int ELEVATOR_DOWN_PORT = 1;

    public final static int ELEVATOR_CARRIAGE_DOWN_PORT = 0;

    // ENCODER INFORMATION
    public final static boolean ELEVATOR_ENCODER_INVERTED = ROBOT_TYPE == Robot.COMPETITION ? false : true;
    public final static boolean ELEVATOR_ENCODER_SENSOR_PHASE = true;

    public final static int DRIVEBASE_ENCODER_SAMPLES_TO_AVERAGE = 7;
    public final static double DRIVEBASE_ENCODER_MIN_RATE = 10;
    public final static double DRIVEBASE_ENCODER_MAX_PERIOD = 0.2;

    public final static double DRIVEBASE_WHEEL_DIAMETER = 6.0; // WHEEL DIAMETER IN INCHES

	// Calcs
    public final static int DRIVEBASE_ENCODER_STEPS_PER_FULL_ROTATION = 500;
    public final static double DRIVEBASE_ENCODER_GEAR_RATIO = ROBOT_TYPE == Robot.PRACTICE ? 84.0 / 44.0 : 84.0 / 54.0;
	// public final static int ENCODER_SAMPLES_TO_AVERAGE = 25; //5 steps to average
	public final static double DRIVEBASE_ENCODER_DISTANCE_PER_PULSE = ((DRIVEBASE_WHEEL_DIAMETER * Math.PI)
/ DRIVEBASE_ENCODER_STEPS_PER_FULL_ROTATION) / DRIVEBASE_ENCODER_GEAR_RATIO;

    public final static int DRIVEBASE_LEFT_ENCODER_PORT_A = 8;
    public final static int DRIVEBASE_LEFT_ENCODER_PORT_B = 9;
    public final static boolean DRIVEBASE_LEFT_ENCODER_INVERTED = ROBOT_TYPE == Robot.PRACTICE ? false : true;

    public final static int DRIVEBASE_RIGHT_ENCODER_PORT_A = 6;
    public final static int DRIVEBASE_RIGHT_ENCODER_PORT_B = 7;
    public final static boolean DRIVEBASE_RIGHT_ENCODER_INVERTED = ROBOT_TYPE == Robot.PRACTICE ? false : true;

    // values

    public final static double ARM_STOW_ANGLE = 171;
    //The angle to go to where it is safe to move the elevator before stowing
    public final static double ARM_PRESTOW_FORWARD_ANGLE = 140;
    public final static double ARM_PRESTOW_REVERSE_ANGLE = 240;

    public final static double ARM_MAX_ANGLE = 280;

    // PID
    public final static PIDConstants DRIVEBASE_DISTANCE_STANDARD_PID = new PIDConstants(0.02, 0.0001, 0.04);
    public final static PIDConstants DRIVEBASE_DISTANCE_MIMIC_PID = new PIDConstants(0.02, 0.0001, 0.04);

    public final static PIDConstants DRIVEBASE_ANGLE_STANDARD_PID = new PIDConstants(0.01, 0.0, 0.0);
    public final static PIDConstants DRIVEBASE_ANGLE_MIMIC_PID = new PIDConstants(0.004, 0.000185, 0.04);

    public final static double AUTO_AIM_DISTANCE_P = 0.01;//gentle: 0.023 rought: 0.01
    public final static double AUTO_AIM_DISTANCE_MIN = 0.24;//gentle: 0.05 rough: 0.2
    public final static double AUTO_AIM_ANGLE_CENTERED_P = 0.005;//0.05
    public final static double AUTO_AIM_ANGLE_UNCENTERED_P = 0.005;
    public final static double AUTO_AIM_ANGLE_MIN = 0.005;

    //motion magic
    public final static PIDConstants ELEVATOR_MM_PID = new PIDConstants(ROBOT_TYPE == Robot.COMPETITION ? 1.2 : 1.5, 0.0, 0.0);
    public final static PIDConstants ELEVATOR_VELOCTIY_PID = new PIDConstants(1, 0.0, 0.0);
    public final static double ELEVATOR_MM_PEAK_SENSOR_VELOCITY = ROBOT_TYPE == Robot.COMPETITION ? 5100 : 5200;
    public final static double ELEVATOR_MM_CRUISE_VELOCITY = 1.0;
    public final static double ELEVATOR_MM_ACCELERATION = 1.4;

    //smart motion
    public final static PIDConstants ARM_SM_PID = new PIDConstants(0.00001, 0.0, 0.001);
    public final static double ARM_SM_PEAK_SENSOR_VELOCITY = 5000;
    public final static double ARM_SM_CRUISE_VELOCITY = 1.0;
    public final static double ARM_SM_ACCELERATION = 1.0;

    // SPEEDS

    // TELEOPERATED SPEEDS
    public final static double ARM_PIVOT_UP_SPEED = 0.2;//0.2
    public final static double ARM_PIVOT_DOWN_SPEED = 0.15;//0.15
    public final static double INTAKE_ROLLER_SPEED = 1.0;

    // TELEOPERATED MULTIPLIERS
    public final static double ELEVATOR_DOWN_MULTIPLIER = 0.6;

    //AUTONOMOUS SPEEDS
    public final static double AUTO_PATHFINDING_OVERRIDE_NEXT_STEP_SPEED = 0.2;

    public final static double AUTO_ROBOT_DEFAULT_SPEED = 0.3;

    public final static double AUTO_ELEVATOR_SPEED = 0.2;//old: 0.4
    public final static double AUTO_ELEVATOR_ACCELERATION = 0.35;// old: 0.35
    public final static double AUTO_ELEVATOR_ACCELERATION_MAX_ERROR = 20000;
    public final static double AUTO_ELEVATOR_STAGE_2_FEEDFORWARD = 0.1;// old: 0.15

    public final static double AUTO_ARM_SPEED = 0.02;//old: 0.15
    public final static double AUTO_ARM_ACCELERATION = 0.9;//old: 0.1
    public final static double AUTO_ARM_ACCELERATION_MAX_ERROR = 120;

    //AUTONOMOUS MULTIPLIERS
    public final static double AUTO_ELEVATOR_DOWN_MULTIPLIER = 0.7;
    public final static double AUTO_ARM_DOWN_MULITPLIER = 0.7;

    // to activate the clutch it runs these
    public final static double ELEVATOR_BRAKE_UP_SPEED = 0.1;
    public final static double ELEVATOR_STALL_SPEED = 0.07;

    // TIMINGS
    public final static long ELEVATOR_BRAKE_TIME = 200;

    // LIMITS
    // The minimum height the elevator needs to be to have the intake flip
    public final static int ELEVATOR_INTAKE_FLIP_LIMIT = (int)(0.4255 * ELEVATOR_ENCODER_CALIBRATION);
    // The minimum height the elevator needs to be in to consider the second stage engaged
    public final static int ELEVATOR_FIRST_STAGE_LIMIT = (int)(0.5106 * ELEVATOR_ENCODER_CALIBRATION);
    // The max operational height of the elevator
    public final static int ELEVATOR_TOP_LIMIT = (int)ELEVATOR_ENCODER_CALIBRATION;
    // The height at which the elevator will be considered at the bottom of it's operational height
    public final static int ELEVATOR_BOTTOM_LIMIT = (int)(0.0127 * ELEVATOR_ENCODER_CALIBRATION);
    // Before going into floor intake mode, the elevator will go to this height to have the intake slowly fall down
    public final static int ELEVATOR_FLOOR_INTAKE_HEIGHT = (int)(0.0638 * ELEVATOR_ENCODER_CALIBRATION);

    // TOLERANCES
    public final static double ARM_ANGLE_TOLERANCE = 0.5;
    public final static int ELEVATOR_POSITION_TOLERANCE = 300;
    public final static double DRIVEBASE_DISTANCE_TOLERNACE = 1;
    public final static double DRIVEBASE_ANGLE_TOLERANCE = 1.0; 

    //AMPS
    public final static int ROLLER_BALL_AMPS = 73;
}