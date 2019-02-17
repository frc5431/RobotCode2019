package frc.robot;

public final class Constants {

    //teleoperated control definitions
    public final static int DRIVER_JOYSTICK_ID = 0;
    public final static double DRIVER_JOYSTICK_DEADZONE = 0.1;

    public final static int OPERATOR_JOYSTICK_ID = 1;
    public final static double OPERATOR_JOYSTICK_DEADZONE = 0.1;

    public final static int BUTTONBOARD_JOYSTICK_ID = 2;

    //pneumatic definitions
    public final static int ELEVATOR_BRAKE_PCM_ID = 30;
    public final static int ELEVATOR_BRAKE_FORWARD_ID = 0;
    public final static int ELEVATOR_BRAKE_REVERSE_ID = 1;

    public final static int ARM_BRAKE_PCM_ID = 31;
    public final static int ARM_BRAKE_ID = 0;
    
    public final static int ARM_WRIST_LEFT_PCM_ID = 31;
    public final static int ARM_WRIST_LEFT_FORWARD_ID = 6;
    public final static int ARM_WRIST_LEFT_REVERSE_ID = 7;

    public final static int ARM_WRIST_RIGHT_PCM_ID = 30;
    public final static int ARM_WRIST_RIGHT_FORWARD_ID = 7;
    public final static int ARM_WRIST_RIGHT_REVERSE_ID = 4;
    
    public final static int INTAKE_HATCH_LEFT_PCM_ID = 31;
    public final static int INTAKE_HATCH_LEFT_FORWARD_ID = 4;
    public final static int INTAKE_HATCH_LEFT_REVERSE_ID = 5;

    public final static int INTAKE_HATCH_RIGHT_PCM_ID = 30;
    public final static int INTAKE_HATCH_RIGHT_FORWARD_ID = 5;
    public final static int INTAKE_HATCH_RIGHT_REVERSE_ID = 3;

    public final static int INTAKE_FINGER_PCM_ID = 30;
    public final static int INTAKE_FINGER_FORWARD_ID = 6;
    public final static int INTAKE_FINGER_REVERSE_ID = 2;

    //motor controller definitions

    //Spark MAX's
    public final static int DRIVEBASE_FRONT_LEFT_ID = 4;
    public final static boolean DRIVEBASE_FRONT_LEFT_INVERTED = false;

    public final static int DRIVEBASE_FRONT_RIGHT_ID = 7;
    public final static boolean DRIVEBASE_FRONT_RIGHT_INVERTED = true;

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
    public final static boolean INTAKE_ROLLER_INVERTED = true;

    public final static int ELEVATOR_BOTTOM_ID = 2;
    public final static boolean ELEVATOR_BOTTOM_INVERTED = true;

    public final static int ELEVATOR_TOP_ID = 3;
    public final static boolean ELEVATOR_TOP_INVERTED = false;

    // sensors

    // analog inputs
    public final static int WRIST_ENCODER_PORT = 3;
    public final static int CLIMBER_ENCODER_PORT = 1;

    // digital inputs

    public final static int ELEVATOR_CARRIAGE_UP_PORT = 4;
    public final static int ELEVATOR_DOWN_1_PORT = 0;
    public final static int ELEVATOR_DOWN_2_PORT = 1;
    public final static int ELEVATOR_UP_1_PORT = 2;
    public final static int ELEVATOR_UP_2_PORT = 3;

    // ENCODER INFORMATION
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

    // SPEEDS

    // TELEOPERATED SPEEDS
    public final static double ARM_PIVOT_UP_SPEED = 0.4;
    public final static double ARM_PIVOT_DOWN_SPEED = 0.3;
    public final static double CLIMBER_SPEED = 0.1;
    public final static double INTAKE_ROLLER_SPEED = 1.0;

    // TELEOPERATED MULTIPLIERS
    public final static double ELEVATOR_DOWN_MULTIPLIER = 0.6;

    //AUTONOMOUS SPEEDS
    public final static double AUTO_ROBOT_DEFAULT_SPEED = 0.3;
    public final static double AUTO_ROBOT_ELEVATOR_SPEED = 0.6;
    public final static double AUTO_ROBOT_ARM_SPEED = 0.4;

    //AUTONOMOUS MULTIPLIERS
    public final static double AUTO_ELEVATOR_DOWN_MULTIPLIER = 0.6;

    // to activate the clutch it runs these
    public final static double ELEVATOR_BRAKE_UP_SPEED = 0.2;
    public final static double ELEVATOR_BRAKE_DOWN_SPEED = 0.0;

    // TIMINGS
    public final static long ELEVATOR_BRAKE_TIME = 200;

    // LIMITS
    public final static int ELEVATOR_TOP_LIMIT = 50000;
    public final static int ELEVATOR_BOTTOM_LIMIT = 600;

    // PID
    public final static double ELEVATOR_PID_P = 0.0;
    public final static double ELEVATOR_PID_I = 0.0;
    public final static double ELEVATOR_PID_D = 0.0;

    public final static double ARM_PID_P = 0.0;
    public final static double ARM_PID_I = 0.0;
    public final static double ARM_PID_D = 0.0;

    // TURNING INFORMATION
    public final static double TURN_PRECISION = 1.0; // Make sure the turn is within the degree
}