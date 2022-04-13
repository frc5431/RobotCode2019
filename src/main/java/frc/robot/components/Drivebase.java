package frc.robot.components;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.team5431.titan.core.joysticks.Xbox;
import frc.team5431.titan.core.misc.Calc;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Drivebase extends SubsystemBase {
    public static enum AutoType{
        COMMANDS, MIMIC, VISION, POINT_TURN
    };

    private final CANSparkMax frontLeft, frontRight, backLeft, backRight;

    public final Encoder leftEncoder, rightEncoder;

    private double leftPower, rightPower;
    private double leftTarget, rightTarget, angleTarget;

    //public PIDController drivePID = new PIDController(0, 0, 0, 0, new DriveBasePIDSource(), new DriveBasePIDOutput());

    // private final PIDController l = new PIDController(1, 0, 0, 0.02);
    // private final PIDController r = new PIDController(1, 0, 0, 0.02);
    // private final PIDController a = new PIDController(1, 0, 0, 0.02);

    private ControlMode controlMode = ControlMode.MANUAL;
    // private AutoType autoType = AutoType.COMMANDS;

    //private final TitanNavx navx;
    private final ADXRS450_Gyro gyro;
    
    public Drivebase(){
        frontLeft = new CANSparkMax(Constants.DRIVEBASE_FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeft.setInverted(Constants.DRIVEBASE_FRONT_LEFT_INVERTED);
        frontLeft.setIdleMode(IdleMode.kBrake);
        frontLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        frontLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        frontLeft.setOpenLoopRampRate(0);
        frontLeft.burnFlash();
        
        frontRight = new CANSparkMax(Constants.DRIVEBASE_FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight.setInverted(Constants.DRIVEBASE_FRONT_RIGHT_INVERTED);
        frontRight.setIdleMode(IdleMode.kBrake);
        frontRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        frontRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        frontRight.setOpenLoopRampRate(0);
        frontRight.burnFlash();
        
        backLeft = new CANSparkMax(Constants.DRIVEBASE_BACK_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backLeft.setInverted(Constants.DRIVEBASE_BACK_LEFT_INVERTED);
        backLeft.follow(ExternalFollower.kFollowerDisabled, 0);
        backLeft.setIdleMode(IdleMode.kBrake);
        backLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        backLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        backLeft.setOpenLoopRampRate(0);
        backLeft.burnFlash();

        //he attac
        //he protec
        //but most importantly
        //he bac
        backRight = new CANSparkMax(Constants.DRIVEBASE_BACK_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backRight.setInverted(Constants.DRIVEBASE_BACK_RIGHT_INVERTED);
        backRight.follow(ExternalFollower.kFollowerDisabled, 0);
        backRight.setIdleMode(IdleMode.kBrake);
        backRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        backRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        backRight.setOpenLoopRampRate(0);
        backRight.burnFlash();

        leftEncoder = new Encoder(Constants.DRIVEBASE_LEFT_ENCODER_PORT_A, Constants.DRIVEBASE_LEFT_ENCODER_PORT_B, false, EncodingType.k4X);
        leftEncoder.setMaxPeriod(Constants.DRIVEBASE_ENCODER_MAX_PERIOD);
        leftEncoder.setMinRate(Constants.DRIVEBASE_ENCODER_MIN_RATE);
        leftEncoder.setDistancePerPulse(Constants.DRIVEBASE_ENCODER_DISTANCE_PER_PULSE);
        leftEncoder.setSamplesToAverage(Constants.DRIVEBASE_ENCODER_SAMPLES_TO_AVERAGE);
        leftEncoder.setReverseDirection(Constants.DRIVEBASE_LEFT_ENCODER_INVERTED);
        
        rightEncoder = new Encoder(Constants.DRIVEBASE_RIGHT_ENCODER_PORT_A, Constants.DRIVEBASE_RIGHT_ENCODER_PORT_B, false, EncodingType.k4X);
        rightEncoder.setMaxPeriod(Constants.DRIVEBASE_ENCODER_MAX_PERIOD);
        rightEncoder.setMinRate(Constants.DRIVEBASE_ENCODER_MIN_RATE);
        rightEncoder.setDistancePerPulse(Constants.DRIVEBASE_ENCODER_DISTANCE_PER_PULSE);
        rightEncoder.setSamplesToAverage(Constants.DRIVEBASE_ENCODER_SAMPLES_TO_AVERAGE);
        rightEncoder.setReverseDirection(Constants.DRIVEBASE_RIGHT_ENCODER_INVERTED);

        disableAllPID();

        //navx = new TitanNavx();

        gyro = new ADXRS450_Gyro();
        resetAll();
    }

    @Override
    public void periodic(){
        //System.out.println(getLeftError() + ", " + getRightError());
        //System.out.println(leftCorrection +", " + leftDistancePID.getError());
        // leftCorrection = 0;
        // rightCorrection = 0;

        final double leftCorrection, rightCorrection, angleCorrection;
        leftCorrection = leftTarget - getLeftDistance();
        rightCorrection = rightTarget - getRightDistance();
        angleCorrection = angleTarget - getAngle();

        final double outLeftPower = MathUtil.clamp(leftPower - leftCorrection - angleCorrection, -1.0, 1.0);
        final double outRightPower = MathUtil.clamp(rightPower - rightCorrection + angleCorrection, -1.0, 1.0);
        backLeft.set(outLeftPower);
        frontLeft.set(outLeftPower);
        backRight.set(outRightPower);
        frontRight.set(outRightPower);

        //System.out.println(getLeftDistance() + ", " + getRightDistance());

        SmartDashboard.putNumber("ADX", gyro.getAngle());

        if(Robot.getRobot().getTeleop().getDriver().getRawButton(Xbox.Button.Y)){
            // navx.enableBoardlevelYawReset(true);
            // navx.reset();
            // navx.resetYaw();
            // navx.zeroYaw();
            // navx.enableBoardlevelYawReset(false);
            // navx.reset();
            // navx.resetYaw();
            //navx.zeroYaw();

            gyro.calibrate();
        }
    }

    public double getLeftPower(){
        return leftPower;
    }

    public double getRightPower(){
        return rightPower;
    }

    public void driveLeft(final double val){
        leftPower = val;
    }

    public void driveRight(final double val){
        rightPower = val;
    }

    public void drive(final double left, final double right){
        driveLeft(left);
        driveRight(right);
    }

    public void resetEncoders(){
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getLeftDistance(){
        return leftEncoder.getDistance();
    }

    public double getRightDistance(){
        return rightEncoder.getDistance();
    }

    public void resetNavx(){
        // navx.reset();
        // navx.resetDisplacement();
        // navx.resetYaw();

        new Thread(()->{
            gyro.reset();
        }).start();
    }

    // public TitanNavx getNavx(){
    //     return navx;
    // }

    public void reset(){
        resetEncoders();
        resetNavx();
    }

    public void disableDistancePID(){
        // leftDistancePID.reset();
        // rightDistancePID.reset();

        setDistancePIDTarget(0, 0);
    }

    public void disableAnglePID(){
        // anglePID.reset();

        setAnglePIDTarget(0);
    }
    
	public final void disableAllPID() {
        disableAnglePID();
        disableDistancePID();
    }

    public void resetAll() {
		reset();
		disableAllPID();
    }

    public void prepareForAutoControl(final AutoType type){
        setControlMode(ControlMode.AUTO);
        // autoType = type;

        resetEncoders();
        disableAllPID();
    }

    public void disableAutoControl(){
        disableAllPID();
        drive(0, 0);
        setControlMode(ControlMode.MANUAL);
    }

    public void enableDistancePID(){
        // final PIDConstants pid;
        // switch(autoType){
        // case VISION:
        //     pid = new PIDConstants(0, 0, 0);
        //     break;
        // case MIMIC:
        //     pid = Constants.DRIVEBASE_DISTANCE_MIMIC_PID;
        //     break;
        // case POINT_TURN:
        // case COMMANDS:
        // default:
        //     pid = Constants.DRIVEBASE_DISTANCE_STANDARD_PID;
        // }
        
        // if(!leftDistancePID.isEnabled()){
        //     leftDistancePID.setPID(pid.getP(), pid.getI(), pid.getD());
        //     leftDistancePID.setInputRange(-1000, 1000);
        //     leftDistancePID.setOutputRange(-1.0, 1.0);
        //     leftDistancePID.setContinuous(false);
        //     leftDistancePID.setAbsoluteTolerance(Constants.DRIVEBASE_DISTANCE_TOLERNACE);
        //     leftDistancePID.setSetpoint(0);
        //     leftDistancePID.enable();
        // }

        // if(!rightDistancePID.isEnabled()){
        //     rightDistancePID.setPID(pid.getP(), pid.getI(), pid.getD());
        //     rightDistancePID.setInputRange(-1000, 1000);
        //     rightDistancePID.setOutputRange(-1.0, 1.0);
        //     rightDistancePID.setContinuous(false);
        //     rightDistancePID.setAbsoluteTolerance(Constants.DRIVEBASE_DISTANCE_TOLERNACE);
        //     rightDistancePID.setSetpoint(0);
        //     rightDistancePID.enable();
        // }
    }

    public void enableAnglePID(){
        // final PIDConstants pid;
        // switch(autoType){
        // case VISION:
        //     pid = new PIDConstants(0, 0, 0);
        //     break;
        // case POINT_TURN:
        //     pid = Constants.DRIVEBASE_ANGLE_POINT_TURN_PID;
        //     break;
        // case MIMIC:
        //     pid = Constants.DRIVEBASE_ANGLE_MIMIC_PID;
        //     break;
        // case COMMANDS:
        // default:
        //     pid = Constants.DRIVEBASE_ANGLE_STANDARD_PID;
        // }

        // if(!anglePID.isEnabled()){
        //     anglePID.setPID(pid.getP(), pid.getI(), pid.getD());
        //     anglePID.setInputRange(-360, 360);
        //     anglePID.setOutputRange(-1.0, 1.0);
        //     anglePID.setContinuous(false);
        //     anglePID.setAbsoluteTolerance(Constants.DRIVEBASE_ANGLE_TOLERANCE);
        //     anglePID.setSetpoint(0);
        //     anglePID.enable();
        // }
    }

    public void setDistancePIDTarget(final double[] setpoints){
        setDistancePIDTarget(setpoints[0], setpoints[1]);
    }

    public void setDistancePIDTarget(final double leftSetpoint, final double rightSetpoint){
        //leftDistancePID.reset();
        //rightDistancePID.reset();
        leftTarget = leftSetpoint;
        rightTarget = rightSetpoint;

       // leftDistancePID.enable();
        //rightDistancePID.enable();
    }

    public void setAnglePIDTarget(final double setpoint){
        angleTarget = setpoint;
    }

    public boolean isAtDistancePIDTarget(){
        return Calc.approxEquals(leftTarget, getLeftDistance(), 0.2) 
            && Calc.approxEquals(rightTarget, getRightDistance(), 0.2);
    }

    public boolean isAtAnglePIDTarget(){
        return Calc.approxEquals(angleTarget, getAngle(), 0.1);
    }

    public double getLeftError(){
        return leftTarget - getLeftDistance();
    }

    public double getRightError(){
        return rightTarget - getRightDistance();
    }

    public double getAngleError(){
        return angleTarget - getAngle();
    }

    public final boolean hasTravelledLeft(final double leftDistance){
        return leftDistance < 0 ? getLeftDistance() <= leftDistance : getLeftDistance() >= leftDistance;
    }

    public final boolean hasTravelledRight(final double rightDistance){
        return rightDistance < 0 ? getRightDistance() <= rightDistance : getRightDistance() >= rightDistance;
    }

    public final boolean hasTravelled(final double leftDistance, final double rightDistance) {
        return hasTravelledRight(rightDistance)
        && hasTravelledLeft(leftDistance);
    }

    public ControlMode getControlMode(){
        return controlMode;
    }

    public void setControlMode(final ControlMode mode){
        controlMode = mode;
    }
	
	public final boolean hasTurned(final double wantedAngle) {
		return wantedAngle < 0 ? getAngle() <= wantedAngle : getAngle() >= wantedAngle;
    }

    public double getAngle(){
        return gyro.getAngle();
    }
}