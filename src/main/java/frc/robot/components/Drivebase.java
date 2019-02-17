package frc.robot.components;

import frc.robot.Constants;
// import frc.robot.Titan;
// import frc.robot.TitanNavx;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class Drivebase {
    private final CANSparkMax frontLeft, frontRight, backLeft, backRight;

    private final Encoder leftEncoder, rightEncoder;

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
    }

    public void driveLeft(final double val){
        frontLeft.set(val);
        //backLeft.set(val);
    }

    public void driveRight(final double val){
        frontRight.set(val);
        //backRight.set(val);
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

    public void setHome() {
		reset();
		//disableAllPID();
    }

    public final boolean hasTravelled(final double wantedDistance) {
		return hasTravelled(wantedDistance, true);
    }

    public final boolean hasTravelled(final double wantedDistance, final boolean isLeft) {
		if (wantedDistance < 0) {
			return ((isLeft) ? getLeftDistance() : getRightDistance()) <= wantedDistance;
		} else {
			return ((isLeft) ? getLeftDistance() : getRightDistance()) >= wantedDistance;
		}
		/*if(wantedDistance < 0) {
			return leftEncoder.getDistance() < wantedDistance || rightEncoder.getDistance() < wantedDistance;
		} else {
			return leftEncoder.getDistance() > wantedDistance || rightEncoder.getDistance() > wantedDistance;
		}*/
	}
	
// 	public final boolean hasTurned(final double wantedAngle) {
// 		return Titan.approxEquals(wantedAngle, navx.getAngle(), Constants.TURN_PRECISION);
// }
}