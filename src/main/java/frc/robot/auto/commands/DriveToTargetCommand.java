package frc.robot.auto.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.components.Vision;
import frc.robot.components.Drivebase;
import frc.robot.auto.vision.TargetInfo;
import frc.robot.auto.vision.TargetType;

public class DriveToTargetCommand extends Titan.Command<Robot> {
	private final TargetType ttype;

	private double lastDistance = 0;
	private long lastDistanceChange = 0;

	private double lastErrorAngle = 0;


	public DriveToTargetCommand(){
		this(TargetType.FRONT_RIGHT);
	}

	public DriveToTargetCommand(final TargetType type){
		this.ttype = type;

		name = "Drive to vision target";
		properties = "Type: " + type.name();
	}

    @Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.setControlMode(ControlMode.AUTO);
	
		robot.getVision().setTargetType(ttype);

		lastDistanceChange = System.currentTimeMillis();
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		final Vision vision = robot.getVision();

		if(drivebase.getControlMode() == ControlMode.MANUAL){
			if(!robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.BUMPER_R) && !robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.BUMPER_L)){
				vision.setLEDState(Vision.LEDState.OFF);
				robot.getAuton().abort(robot);
				return CommandResult.CLEAR_QUEUE;
			}else{
				drivebase.setControlMode(ControlMode.AUTO);
			}
		}

		vision.setLEDState(Vision.LEDState.ON);

		final double currentDistance = (drivebase.getLeftDistance() + drivebase.getRightDistance()) / 2.0;
		if(currentDistance != lastDistance){
			lastDistance = currentDistance;
			lastDistanceChange = System.currentTimeMillis();
		}

		final double directionSignum = ttype.getLimelight().isInverted() ? -1 : 1;

		/*
		When the elevator is running, to avoid breaking the carriage, we want to slow down.
		*/
		final boolean isRunningElevator = !Titan.approxEquals(robot.getElevator().getEncoderVelocity(), 0, 200);

		final TargetInfo target = vision.getTargetInfo();
		/*
		When in reverse, the angles are flipped
		*/
		//double angleError = directionSignum * target.getXAngle();
		double angleError;
		if(target.exists()){
			if(ttype.getLimelight().isCentered()){
				angleError = target.getXAngle();
			}else{
				angleError = (target.getXAngle() / target.getYAngle()) - 1.18;
			}
			lastErrorAngle = angleError;
		}else{
			angleError = lastErrorAngle;
		}
		angleError *= directionSignum;
		if(Titan.approxEquals(angleError, 1, 0.1)){
			angleError = 0;
		}
		final double distanceError = target.getYAngle();

		// you are allowed to be too close, as the intake will just ram the hatch into the rocket
		if((target.exists() && distanceError < 0) || (!isRunningElevator && System.currentTimeMillis() > lastDistanceChange + 500)){
			robot.getDrivebase().drive(0, 0);

			robot.getVision().setLEDState(Vision.LEDState.OFF);
			return CommandResult.COMPLETE;
		}

		double rawPower = directionSignum * (isRunningElevator ? 0.0 : 0.2 + (Constants.AUTO_AIM_DISTANCE_P * distanceError));
		rawPower *= (5.0 - Math.min(Math.abs(angleError), 5)) / 5.0;
		final double angleAdjust = (isRunningElevator ? 0.0 : (Constants.AUTO_AIM_ANGLE_P * angleError)) + (Math.signum(angleError) * Constants.AUTO_AIM_ANGLE_MIN);

		drivebase.drive(rawPower + angleAdjust, rawPower - angleAdjust);
        
		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void done(final Robot robot) {
	}
}