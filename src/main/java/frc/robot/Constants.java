package frc.robot;

public final class Constants {
    private static enum Robot {
        PRACTICE, COMPETITION;
    }

    public static final Robot ROBOT_TYPE = Robot.PRACTICE;

    //teleoperated control definitions
    public final static int DRIVER_JOYSTICK_ID = 0;
    public final static double DRIVER_JOYSTICK_DEADZONE = 0.1;

    public final static int OPERATOR_JOYSTICK_ID = 1;
    public final static double OPERATOR_JOYSTICK_DEADZONE = 0.1;

    public final static int BUTTONBOARD_JOYSTICK_ID = 2;

    //pneumatic definitions
    public final static int ELEVATOR_BRAKE_PCM_ID = 30;
    public final static int ELEVATOR_BRAKE_ID = 0;
    //public final static int ELEVATOR_BRAKE_REVERSE_ID = 1;

    public final static int ARM_BRAKE_PCM_ID = 30;
    public final static int ARM_BRAKE_ID = ROBOT_TYPE == Robot.COMPETITION ? 7 : 1;
    
    // public final static int ARM_WRIST_PCM_ID = 30;
    // public final static int ARM_WRIST_ID = 5;
    //public final static int ARM_WRIST_LEFT_REVERSE_ID = 7;

    //public final static int ARM_WRIST_RIGHT_REVERSE_ID = 4;
    
    public final static int INTAKE_HATCH_PCM_ID = 30;
    public final static int INTAKE_HATCH_ID = 3;

    public final static int INTAKE_JAY_PCM_ID = 30;
    public final static int INTAKE_JAY_ID = 4;
    //public final static int INTAKE_HATCH_LEFT_REVERSE_ID = 5;

    //public final static int INTAKE_HATCH_RIGHT_PCM_ID = 30;
    //public final static int INTAKE_HATCH_RIGHT_ID = 5;
    //public final static int INTAKE_HATCH_RIGHT_REVERSE_ID = 3;

    public final static int INTAKE_FINGER_PCM_ID = 30;
    public final static int INTAKE_FINGER_ID = 6;
    //public final static int INTAKE_FINGER_REVERSE_ID = 2;

    //motor controller definitions

    //Spark MAX's
    public final static int DRIVEBASE_FRONT_LEFT_ID = 4;
    public final static boolean DRIVEBASE_FRONT_LEFT_INVERTED = true;

    public final static int DRIVEBASE_FRONT_RIGHT_ID = 7;
    public final static boolean DRIVEBASE_FRONT_RIGHT_INVERTED = false;

    // NOTE: The back motor follows the front so keep that in mind when inverting

    public final static int DRIVEBASE_BACK_LEFT_ID = 5;
    public final static boolean DRIVEBASE_BACK_LEFT_INVERTED = false;
    
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
    public final static boolean INTAKE_ROLLER_INVERTED = false;

    public final static int ELEVATOR_BOTTOM_ID = 3;
    public final static boolean ELEVATOR_BOTTOM_INVERTED = false;

    public final static int ELEVATOR_TOP_ID = 2;
    public final static boolean ELEVATOR_TOP_INVERTED = true;

    // sensors

    // analog inputs
    public final static int ARM_ENCODER_PORT = 3;

    public final static double ARM_ENCODER_CALIBRATION_OFFSET = ROBOT_TYPE == Robot.COMPETITION ? 74 : 178;

    // digital inputs

    public final static int ELEVATOR_CARRIAGE_UP_PORT = 4;
    public final static int ELEVATOR_DOWN_1_PORT = 1;
    //public final static int ELEVATOR_DOWN_2_PORT = 3;

    public final static int ELEVATOR_CARRIAGE_DOWN_1_PORT = 0;
    //public final static int ELEVATOR_CARRIAGE_DOWN_2_PORT = 5;

    // ENCODER INFORMATION
    public final static boolean ELEVATOR_ENCODER_INVERTED = ROBOT_TYPE == Robot.COMPETITION ? false : true;

    public final static int DRIVEBASE_ENCODER_SAMPLES_TO_AVERAGE = 7;
    public final static double DRIVEBASE_ENCODER_MIN_RATE = 10;
    public final static double DRIVEBASE_ENCODER_MAX_PERIOD = 0.2;

    public final static double DRIVEBASE_WHEEL_DIAMETER = 6.0; // WHEEL DIAMETER IN INCHES

	// Calcs
	public final static int DRIVEBASE_ENCODER_STEPS_PER_FULL_ROTATION = 500;
	// public final static int ENCODER_SAMPLES_TO_AVERAGE = 25; //5 steps to average
	public final static double DRIVEBASE_ENCODER_DISTANCE_PER_PULSE = (DRIVEBASE_WHEEL_DIAMETER * Math.PI)
/ DRIVEBASE_ENCODER_STEPS_PER_FULL_ROTATION;

    public final static int DRIVEBASE_LEFT_ENCODER_PORT_A = 8;
    public final static int DRIVEBASE_LEFT_ENCODER_PORT_B = 9;
    public final static boolean DRIVEBASE_LEFT_ENCODER_INVERTED = false;

    public final static int DRIVEBASE_RIGHT_ENCODER_PORT_A = 6;
    public final static int DRIVEBASE_RIGHT_ENCODER_PORT_B = 7;
    public final static boolean DRIVEBASE_RIGHT_ENCODER_INVERTED = false;

    // values

    public final static double ARM_STOW_ANGLE = 169;
    //The angle to go to where it is safe to move the elevator before stowing
    public final static double ARM_PRESTOW_FORWARD_ANGLE = 140;
    public final static double ARM_PRESTOW_REVERSE_ANGLE = 220;

    // PID
    public final static double AUTO_DISTANCE_P = 0.00635;//0.00635
	public final static double AUTO_DISTANCE_I = 0.00000000025; // 0.0003; //0.0022;
    public final static double AUTO_DISTANCE_D = 0.000915; // 0.00031;

    // public final static double AUTO_DISTANCE_P = 0.002;
	// public final static double AUTO_DISTANCE_I = 0.0; // 0.0003; //0.0022;
    // public final static double AUTO_DISTANCE_D = 0.0; // 0.00031;


    // 115, 85

    public final static double DRIVE_MIMICK_P = 0.016; //0.025
	public final static double DRIVE_MIMICK_I = 0.00;
	public final static double DRIVE_MIMICK_D = 0.060;
    public final static double DRIVE_MIMICK_MIN_MAX = 0.35; //0.4

    // SPEEDS

    // TELEOPERATED SPEEDS
    public final static double ARM_PIVOT_UP_SPEED = 0.2;
    public final static double ARM_PIVOT_DOWN_SPEED = 0.15;
    public final static double CLIMBER_SPEED = 0.1;
    public final static double INTAKE_ROLLER_SPEED = 1.0;

    // TELEOPERATED MULTIPLIERS
    public final static double ELEVATOR_DOWN_MULTIPLIER = 0.6;

    //AUTONOMOUS SPEEDS
    public final static double AUTO_PATHFINDING_OVERRIDE_NEXT_STEP_SPEED = 0.2;

    public final static double AUTO_ROBOT_DEFAULT_SPEED = 0.3;

    public final static double AUTO_ELEVATOR_SPEED = 0.5;//old: 0.4
    public final static double AUTO_ELEVATOR_ACCELERATION = 0.35;// old: 0.35
    public final static double AUTO_ELEVATOR_ACCELERATION_MAX_ERROR = 20000;
    public final static double AUTO_ELEVATOR_STAGE_2_FEEDFORWARD = 0.1;// old: 0.15

    public final static double AUTO_ARM_SPEED = 0.1;//old: 0.15
    public final static double AUTO_ARM_ACCELERATION = 0.9;//old: 0.1
    public final static double AUTO_ARM_ACCELERATION_MAX_ERROR = 120;

    //AUTONOMOUS MULTIPLIERS
    public final static double AUTO_ELEVATOR_DOWN_MULTIPLIER = 0.7;
    public final static double AUTO_ARM_DOWN_MULITPLIER = 0.7;

    // to activate the clutch it runs these
    public final static double ELEVATOR_BRAKE_UP_SPEED = 0.2;

    // TIMINGS
    public final static long ELEVATOR_BRAKE_TIME = 200;

    // LIMITS
    // The minimum height the elevator needs to be to have the intake flip
    public final static int ELEVATOR_INTAKE_FLIP_LIMIT = 23000;
    // The minimum height the elevator needs to be in to consider the second stage engaged
    public final static int ELEVATOR_FIRST_STAGE_LIMIT = 24000;
    // The max operational height of the elevator
    public final static int ELEVATOR_TOP_LIMIT = 50000;
    // The height at which the elevator will be considered at the bottom of it's operational height
    public final static int ELEVATOR_BOTTOM_LIMIT = 600;
    // Before going into floor intake mode, the elevator will go to this height to have the intake slowly fall down
    public final static int ELEVATOR_FLOOR_INTAKE_HEIGHT = 3000;

    // TOLERANCES
    public final static double ARM_ANGLE_TOLERANCE = 0.5;
    public final static int ELEVATOR_POSITION_TOLERANCE = 750;

    //AMPS
    public final static int ROLLER_BALL_AMPS = 73;
}