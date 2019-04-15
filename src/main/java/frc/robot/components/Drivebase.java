package frc.robot.components;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.util.PIDConstants;
import frc.robot.util.Titan;
import frc.robot.util.TitanNavx;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivebase extends Titan.Component<Robot>{
    public static enum AutoType{
        COMMANDS, MIMIC, VISION, POINT_TURN, DRIVE_TO
    };

    private final CANSparkMax frontLeft, frontRight, backLeft, backRight;

    private final Encoder leftEncoder, rightEncoder;

    private double leftPower, rightPower;
    private double leftTarget, rightTarget, angleTarget;

    //public PIDController drivePID = new PIDController(0, 0, 0, 0, new DriveBasePIDSource(), new DriveBasePIDOutput());

    private final PIDController leftDistancePID = new PIDController(0, 0, 0, new PIDSource(){

        
        @Override
        public void setPIDSourceType(final PIDSourceType pidSource) {
            //do nothing
        }
    
        @Override
        public double pidGet() {
            return leftTarget - getLeftDistance();
        }
    
        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }
    }, new PIDOutput() {
		@Override
		public void pidWrite(final double output) {}
    }, 0.02);

    private final PIDController rightDistancePID = new PIDController(0, 0, 0, new PIDSource(){

        
        @Override
        public void setPIDSourceType(final PIDSourceType pidSource) {
            //do nothing
        }
    
        @Override
        public double pidGet() {
            return rightTarget - getRightDistance();
        }
    
        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }
    }, new PIDOutput() {
		@Override
		public void pidWrite(final double output) {}
    }, 0.02);

    private final PIDController anglePID = new PIDController(0, 0, 0, new PIDSource(){

        
        @Override
        public void setPIDSourceType(final PIDSourceType pidSource) {
            //do nothing
        }
    
        @Override
        public double pidGet() {
            return angleTarget - getAngle();
        }
    
        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }
    }, new PIDOutput() {
		@Override
		public void pidWrite(final double output) {}
    }, 0.02);

    private ControlMode controlMode = ControlMode.MANUAL;
    private AutoType autoType = AutoType.COMMANDS;

    private final TitanNavx navx;
    
    public Drivebase(){
        frontLeft = new CANSparkMax(Constants.DRIVEBASE_FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeft.setInverted(Constants.DRIVEBASE_FRONT_LEFT_INVERTED);
        frontLeft.setIdleMode(IdleMode.kBrake);
        frontLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        frontLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        frontLeft.burnFlash();
        
        frontRight = new CANSparkMax(Constants.DRIVEBASE_FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight.setInverted(Constants.DRIVEBASE_FRONT_RIGHT_INVERTED);
        frontRight.setIdleMode(IdleMode.kBrake);
        frontRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        frontRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        frontRight.burnFlash();
        
        backLeft = new CANSparkMax(Constants.DRIVEBASE_BACK_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backLeft.setInverted(Constants.DRIVEBASE_BACK_LEFT_INVERTED);
        backLeft.follow(ExternalFollower.kFollowerDisabled, 0);
        backLeft.setIdleMode(IdleMode.kBrake);
        backLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        backLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
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

        navx = new TitanNavx();
    }

    
    @Override
    public void init(final Robot robot){
        resetAll();
    }

    @Override
    public void periodic(final Robot robot){
        //System.out.println(getLeftError() + ", " + getRightError());
        //System.out.println(leftCorrection +", " + leftDistancePID.getError());
        // leftCorrection = 0;
        // rightCorrection = 0;

        final double leftCorrection, rightCorrection, angleCorrection;
        if(leftDistancePID.isEnabled()){
            leftCorrection = leftDistancePID.get();
        }else{
            leftCorrection = 0;
        }
        if(rightDistancePID.isEnabled()){
            rightCorrection = rightDistancePID.get();
        }else{
            rightCorrection = 0;
        }
        if(anglePID.isEnabled()){
            angleCorrection = anglePID.get();
            //angleCorrection = 0;
        }else{
            angleCorrection = 0;
        }

        final double outLeftPower = leftPower - leftCorrection - angleCorrection;
        final double outRightPower = rightPower - rightCorrection + angleCorrection;
        backLeft.set(outLeftPower);
        frontLeft.set(outLeftPower);
        backRight.set(outRightPower);
        frontRight.set(outRightPower);

        SmartDashboard.putNumber("RightPower", outRightPower);
        SmartDashboard.putNumber("LeftPower", outLeftPower);

        System.out.println(navx.isConnected() + ", " + navx.isCalibrating() + ", " + navx.isMagneticDisturbance());
        //System.out.println(getLeftDistance() + ", " + getRightDistance());

        if(autoType == AutoType.COMMANDS && controlMode == ControlMode.AUTO && leftPower != 0){
            frontLeft.setOpenLoopRampRate(0.7);
            backLeft.setOpenLoopRampRate(0.7);
        }else{
            frontLeft.setOpenLoopRampRate(0);
            backLeft.setOpenLoopRampRate(0);
        }

        if(autoType == AutoType.COMMANDS && controlMode == ControlMode.AUTO && rightPower != 0){
            frontRight.setOpenLoopRampRate(0.7);
            backRight.setOpenLoopRampRate(0.7);
        }else{
            frontRight.setOpenLoopRampRate(0);
            backRight.setOpenLoopRampRate(0);
        }
    }

    @Override
    public void disabled(final Robot robot){
        
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
        navx.reset();
        navx.resetDisplacement();
        navx.resetYaw();
    }

    public TitanNavx getNavx(){
        return navx;
    }

    public void reset(){
        resetEncoders();
        resetNavx();
    }

    public void disableDistancePID(){
        leftDistancePID.reset();
        rightDistancePID.reset();

        setDistancePIDTarget(0, 0);
    }

    public void disableAnglePID(){
        anglePID.reset();

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
        autoType = type;

        resetEncoders();
        disableAllPID();
    }

    public void disableAutoControl(){
        disableAllPID();
        drive(0, 0);
        setControlMode(ControlMode.MANUAL);
    }

    public void enableDistancePID(){
        final PIDConstants pid;
        switch(autoType){
        case VISION:
            pid = new PIDConstants(0, 0, 0);
            break;
        case MIMIC:
            pid = Constants.DRIVEBASE_DISTANCE_MIMIC_PID;
            break;
        case POINT_TURN:
        case COMMANDS:
        default:
            pid = Constants.DRIVEBASE_DISTANCE_STANDARD_PID;
        }
        
        if(!leftDistancePID.isEnabled()){
            leftDistancePID.setPID(pid.getP(), pid.getI(), pid.getD());
            leftDistancePID.setInputRange(-1000, 1000);
            leftDistancePID.setOutputRange(-1.0, 1.0);
            leftDistancePID.setContinuous(false);
            leftDistancePID.setAbsoluteTolerance(Constants.DRIVEBASE_DISTANCE_TOLERNACE);
            leftDistancePID.setSetpoint(0);
            leftDistancePID.enable();
        }

        if(!rightDistancePID.isEnabled()){
            rightDistancePID.setPID(pid.getP(), pid.getI(), pid.getD());
            rightDistancePID.setInputRange(-1000, 1000);
            rightDistancePID.setOutputRange(-1.0, 1.0);
            rightDistancePID.setContinuous(false);
            rightDistancePID.setAbsoluteTolerance(Constants.DRIVEBASE_DISTANCE_TOLERNACE);
            rightDistancePID.setSetpoint(0);
            rightDistancePID.enable();
        }
    }

    public void enableAnglePID(){
        final PIDConstants pid;
        switch(autoType){
        case VISION:
            pid = new PIDConstants(0, 0, 0);
            break;
        case POINT_TURN:
            pid = Constants.DRIVEBASE_ANGLE_POINT_TURN_PID;
            break;
        case MIMIC:
            pid = Constants.DRIVEBASE_ANGLE_MIMIC_PID;
            break;
        case COMMANDS:
        default:
            pid = Constants.DRIVEBASE_ANGLE_STANDARD_PID;
        }

        if(!anglePID.isEnabled()){
            anglePID.setPID(pid.getP(), pid.getI(), pid.getD());
            anglePID.setInputRange(-360, 360);
            anglePID.setOutputRange(-1.0, 1.0);
            anglePID.setContinuous(false);
            anglePID.setAbsoluteTolerance(Constants.DRIVEBASE_ANGLE_TOLERANCE);
            anglePID.setSetpoint(0);
            anglePID.enable();
        }
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
        return leftDistancePID.onTarget() && rightDistancePID.onTarget();
    }

    public boolean isAtAnglePIDTarget(){
        return anglePID.onTarget();
    }

    public double getLeftError(){
        return leftDistancePID.getError();
    }

    public double getRightError(){
        return rightDistancePID.getError();
    }

    public double getAngleError(){
        return anglePID.getError();
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
        return navx.getAngle();
    }
}