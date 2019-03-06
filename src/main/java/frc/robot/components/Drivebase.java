package frc.robot.components;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;
import frc.robot.util.Component;
import frc.robot.util.Testable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class Drivebase extends Component{
    // public class DriveBasePIDSource implements PIDSource {
	// 	@Override
	// 	public void setPIDSourceType(final PIDSourceType pidSource) {
	// 		//do nothing
	// 	}

	// 	@Override
	// 	public PIDSourceType getPIDSourceType() {
	// 		return PIDSourceType.kDisplacement;
	// 	}

	// 	@Override
	// 	public double pidGet() {
	// 		return navx.getAngle();
	// 	}
    // }


    private final CANSparkMax frontLeft, frontRight, backLeft, backRight;

    private final Encoder leftEncoder, rightEncoder;

    private double leftPower, rightPower;

    private double leftCorrection, rightCorrection;

    //public PIDController drivePID = new PIDController(0, 0, 0, 0, new DriveBasePIDSource(), new DriveBasePIDOutput());

    private final PIDController leftDistancePID = new PIDController(Constants.AUTO_DISTANCE_P, Constants.AUTO_DISTANCE_I, Constants.AUTO_DISTANCE_D, new PIDSource(){

        
        @Override
        public void setPIDSourceType(final PIDSourceType pidSource) {
            //do nothing
        }
    
        @Override
        public double pidGet() {
            return getLeftDistance();
        }
    
        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }
    }, new PIDOutput() {
		@Override
		public void pidWrite(final double output) {
            leftCorrection = output;
		}
    }, 0.2);

    private final PIDController rightDistancePID = new PIDController(Constants.AUTO_DISTANCE_P, Constants.AUTO_DISTANCE_I, Constants.AUTO_DISTANCE_D, new PIDSource(){

        
        @Override
        public void setPIDSourceType(final PIDSourceType pidSource) {
            //do nothing
        }
    
        @Override
        public double pidGet() {
            return getRightDistance();
        }
    
        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }
    }, new PIDOutput() {
		@Override
		public void pidWrite(final double output) {
            rightCorrection = output;
		}
    }, 0.2);

    private ControlMode controlMode = ControlMode.MANUAL;

    //private final TitanNavx navx;
    
    public Drivebase(){
        frontLeft = new CANSparkMax(Constants.DRIVEBASE_FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeft.setInverted(Constants.DRIVEBASE_FRONT_LEFT_INVERTED);
        frontLeft.setIdleMode(IdleMode.kBrake);
        
        frontRight = new CANSparkMax(Constants.DRIVEBASE_FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight.setInverted(Constants.DRIVEBASE_FRONT_RIGHT_INVERTED);
        frontRight.setIdleMode(IdleMode.kBrake);
        
        backLeft = new CANSparkMax(Constants.DRIVEBASE_BACK_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backLeft.setInverted(Constants.DRIVEBASE_BACK_LEFT_INVERTED);
        backLeft.follow(frontLeft, Constants.DRIVEBASE_BACK_LEFT_INVERTED);
        backLeft.setIdleMode(IdleMode.kBrake);

        //he attac
        //he protec
        //but most importantly
        //he bac
        backRight = new CANSparkMax(Constants.DRIVEBASE_BACK_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backRight.setInverted(Constants.DRIVEBASE_BACK_RIGHT_INVERTED);
        backRight.follow(frontRight, Constants.DRIVEBASE_BACK_RIGHT_INVERTED);
        backRight.setIdleMode(IdleMode.kBrake);

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
    }

    
    @Override
    public void init(final Robot robot){
        
    }

    @Override
    public void periodic(final Robot robot){
        //System.out.println(leftCorrection +", " + leftDistancePID.getError());
        // leftCorrection = 0;
        // rightCorrection = 0;
        frontLeft.set(leftPower + leftCorrection);
        frontRight.set(rightPower + rightCorrection);
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

    // public void resetNavx(){
    //     navx.reset();
    //     navx.resetDisplacement();
    //     navx.resetYaw();
    // }

    // public TitanNavx getNavx(){
    //     return navx;
    // }

    public void reset(){
        resetEncoders();
        //resetNavx();
    }

    public final void disableDistancePID(){
        leftDistancePID.disable();
        rightDistancePID.disable();
    }
    
	public final void disableAllPID() {
        //drivePID.disable();
        disableDistancePID();
    }

    public void setHome() {
		reset();
		disableAllPID();
    }

    public void enableDistancePID(){
        if(!leftDistancePID.isEnabled()){
            leftDistancePID.setInputRange(-1000, 1000);
            leftDistancePID.setOutputRange(-1.0, 1.0);
            leftDistancePID.setContinuous(false);
            leftDistancePID.setAbsoluteTolerance(3);
            leftDistancePID.enable();
        }

        if(!rightDistancePID.isEnabled()){
            rightDistancePID.setInputRange(-1000, 1000);
            rightDistancePID.setOutputRange(-1.0, 1.0);
            rightDistancePID.setContinuous(false);
            rightDistancePID.setAbsoluteTolerance(3);
            rightDistancePID.enable();
        }
    }

    public void setDistancePIDTarget(final double leftSetpoint, final double rightSetpoint){
        //leftDistancePID.reset();
        //rightDistancePID.reset();
        
        leftDistancePID.setSetpoint(leftSetpoint);
        rightDistancePID.setSetpoint(rightSetpoint);

       // leftDistancePID.enable();
        //rightDistancePID.enable();
    }

    public boolean isAtDistancePIDTarget(){
        return leftDistancePID.onTarget() && rightDistancePID.onTarget();
    }

    // public final void setDrivePIDValues() {
    //     drivePID.setPID(Constants.DRIVE_MIMICK_P, Constants.DRIVE_MIMICK_I, Constants.DRIVE_MIMICK_D, 0.0);
    //     drivePID.setOutputRange(-Constants.DRIVE_MIMICK_MIN_MAX, Constants.DRIVE_MIMICK_MIN_MAX);
    // }

    // public final void driveAtAnglePID(final double leftSpeed, final double rightSpeed, final double angle) {
    //     disableAllPID();
    //     reset();

    //     drive(leftSpeed, rightSpeed);
		
	// 	drivePID.reset();
	// 	drivePID.setSetpoint(0);
	// 	setDrivePIDValues();
	// 	//drivePID.setOutputRange(-Constants.DRIVE_HEADING_MIN_MAX, Constants.DRIVE_HEADING_MIN_MAX);
	// 	drivePID.setSetpoint(angle);
	// 	drivePID.enable();
    // }

    // public final void updateStepResults(final double leftSpeed, final double rightSpeed, final double angle) {
    //     drive(leftSpeed, rightSpeed);

    //     drivePID.setSetpoint(angle);
    // }

    // public final boolean hasTravelled(final double leftDistance, final double rightDistance) {
    //     return rightDistance < 0 ? getRightDistance() <= rightDistance : getRightDistance() >= rightDistance
    //     && leftDistance < 0 ? getLeftDistance() <= leftDistance : getLeftDistance() >= leftDistance;
    // }

    public ControlMode getControlMode(){
        return controlMode;
    }

    public void setControlMode(final ControlMode mode){
        controlMode = mode;
    }
	
// 	public final boolean hasTurned(final double wantedAngle) {
// 		return Titan.approxEquals(wantedAngle, navx.getAngle(), Constants.TURN_PRECISION);
// }

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}