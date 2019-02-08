package frc.robot;

public final class Constants {
    //speed definitions
    public final static double ARM_PIVOT_SPEED = 0.3;
    public final static double CLIMBER_SPEED = 0.1;
    public final static double INTAKE_ROLLER_SPEED = 0.3;

    //teleoperated control definitions
    public final static int DRIVER_JOYSTICK_ID = 0;
    public final static double DRIVER_JOYSTICK_DEADZONE = 0.15;

    public final static int OPERATOR_JOYSTICK_ID = 1;
    public final static double OPERATOR_JOYSTICK_DEADZONE = 0.15;

    //pneumatic definitions
    public final static int ELEVATOR_BRAKE_FORWARD_ID = 0;
    public final static int ELEVATOR_BRAKE_REVERSE_ID = 1;

    public final static int ARM_BRAKE_ID = 2;
    
    public final static int ARM_WRIST_FORWARD_ID = 3;
    public final static int ARM_WRIST_REVERSE_ID = 4;
    
    public final static int INTAKE_HATCH_FORWARD_ID = 5;
    public final static int INTAKE_HATCH_REVERSE_ID = 6;

    //motor controller definitions

    //Spark MAX's
    public final static int DRIVEBASE_FRONT_LEFT_ID = 6;
    public final static boolean DRIVEBASE_FRONT_LEFT_INVERTED = false;

    public final static int DRIVEBASE_FRONT_RIGHT_ID = 5;
    public final static boolean DRIVEBASE_FRONT_RIGHT_INVERTED = true;

    public final static int DRIVEBASE_BACK_LEFT_ID = 7;
    public final static boolean DRIVEBASE_BACK_LEFT_INVERTED = false;
    
    public final static int DRIVEBASE_BACK_RIGHT_ID = 4;
    public final static boolean DRIVEBASE_BACK_RIGHT_INVERTED = true;

    public final static int CLIMBER_LEFT_ID = 4;
    public final static boolean CLIMBER_LEFT_INVERTED = true;

    public final static int CLIMBER_RIGHT_ID = 10;
    public final static boolean CLIMBER_RIGHT_INVERTED = false;

    public final static int ARM_PIVOT_ID = 10;
    public final static boolean ARM_PIVOT_INVERTED = false;

    // Victor SPX's
    public final static int INTAKE_ROLLER_ID = 8;
    public final static boolean INTAKE_ROLLER_INVERTED = false;

    // Talon SRX's
    public final static int ELEVATOR_LEFT_ID = 9;
    public final static boolean ELEVATOR_LEFT_INVERTED = false;

    public final static int ELEVATOR_RIGHT_ID = 10;
    public final static boolean ELEVATOR_RIGHT_INVERTED = false;

    // sensors

    // analog inputs
    public final static int ELEVATOR_ENCODER_PORT = 0;
    public final static int CLIMBER_ENCODER_PORT = 1;

    // digital inputs
    public final static int DRIVEBASE_LEFT_ENCODER_PORT_A = 0;
    public final static int DRIVEBASE_LEFT_ENCODER_PORT_B = 1;

    public final static int DRIVEBASE_RIGHT_ENCODER_PORT_A = 2;
    public final static int DRIVEBASE_RIGHT_ENCODER_PORT_B = 3;
}