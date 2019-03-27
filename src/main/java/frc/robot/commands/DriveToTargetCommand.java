package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.components.Vision;
import frc.robot.components.Drivebase;

public class DriveToTargetCommand extends Titan.Command<Robot> {
	private final Vision.TargetingDirection direction;

	private double lastDistance = 0;
	private long lastDistanceChange = 0;

	private double lastErrorAngle = 0;


	public DriveToTargetCommand(){
		this(Vision.TargetingDirection.FRONT);
	}

	public DriveToTargetCommand(final Vision.TargetingDirection direction){
		this.direction = direction;

		name = "Drive to vision target";
	}

    @Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.setControlMode(ControlMode.AUTO);
	
		robot.getVision().setDirection(direction);

		lastDistanceChange = System.currentTimeMillis();
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		final Vision vision = robot.getVision();

		if(drivebase.getControlMode() == ControlMode.MANUAL){
			vision.setLEDState(Vision.LEDState.OFF);
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		vision.setLEDState(Vision.LEDState.ON);

		final double currentDistance = (drivebase.getLeftDistance() + drivebase.getRightDistance()) / 2.0;
		if(currentDistance != lastDistance){
			lastDistance = currentDistance;
			lastDistanceChange = System.currentTimeMillis();
		}

		final Vision.TargetInfo target = vision.getTargetInfo();
		/*
		Cheeky little trick right here to make the code smaller.
		FRONT's ordinal is 0
		BACK's ordinal is 1

		-1^0 = 1
		-1^1 = -1

		Thus, when it is FRONT, directionSignum is 1. When it is BACK, directionSignum is -1
		*/
		final double directionSignum = Math.pow(-1, direction.ordinal());

		/*
		When the elevator is running, to avoid breaking the carriage, we want to slow down.
		*/
		final boolean isRunningElevator = Titan.approxEquals(robot.getElevator().getEncoderVelocity(), 0, 5);

		/*
		When in reverse, the angles are flipped
		*/
		//double angleError = directionSignum * target.getXAngle();
		double angleError;
		if(target.exists()){
			angleError = (target.getXAngle() / target.getYAngle()) - 1;
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
		if(distanceError < 0 || (angleError == 0 && System.currentTimeMillis() > lastDistanceChange + 500))
		
		{
			robot.getDrivebase().drive(0, 0);

			robot.getVision().setLEDState(Vision.LEDState.OFF);
			return CommandResult.COMPLETE;
		}

		final double rawPower = directionSignum * (isRunningElevator ? 0.0 : 0.3);
		final double angleAdjust = (isRunningElevator ? 0.0 : (Constants.AUTO_AIM_ANGLE_P * angleError)) + (Math.signum(angleError) * Constants.AUTO_AIM_ANGLE_MIN);

		drivebase.drive(rawPower + angleAdjust, rawPower - angleAdjust);
        
		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void done(final Robot robot) {
	}
}