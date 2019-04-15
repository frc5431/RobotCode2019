package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.auto.vision.TargetInfo;
import frc.robot.auto.vision.TargetType;
import frc.robot.components.Drivebase;
import frc.robot.components.Vision;
import frc.robot.components.Drivebase.AutoType;

public class TurnToTargetCommand extends Titan.Command<Robot> {
	private final double speed;
	private final TargetType type;

	private long hitTarget = -1;

	public TurnToTargetCommand(final double speed, final TargetType ttype) {
		name = "TurnToTargetCommand";

		this.type = ttype;
		this.speed = speed;

		properties = String.format("Speed: %f; Target: %s", speed, ttype);
	}

	public TurnToTargetCommand(final TargetType ttype){
		this(Constants.AUTO_TURN_MIN_SPEED, ttype);
	}

	@Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.prepareForAutoControl(AutoType.POINT_TURN);

		drivebase.enableAnglePID();
		drivebase.setAnglePIDTarget(0);

		robot.getVision().setTargetType(type);

		// drivebase.enableDistancePID();
		// drivebase.setDistancePIDTarget(leftDistance, rightDistance);
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

		final double currentAngle = drivebase.getAngle();

		final TargetInfo target = robot.getVision().getTargetInfo();
		final double angle = currentAngle + target.getXAngle();
		drivebase.setAnglePIDTarget(angle);


		final double targetSignum;
		switch(type){
		case FRONT_RIGHT:
			targetSignum = -1;
			break;
		case FRONT_LEFT:
		default:
			targetSignum = 1;
		}

		drivebase.drive(speed * targetSignum * Math.signum(angle - currentAngle), -speed * targetSignum * Math.signum(angle - currentAngle));
		
		if(Titan.approxEquals(currentAngle, angle, Constants.DRIVEBASE_ANGLE_TOLERANCE)){
			if(hitTarget <= 0){
				hitTarget = System.currentTimeMillis();
			}
		}else{
			hitTarget = -1;
		}

		if(/*drivebase.hasTurned(angle)*/hitTarget > 0 && System.currentTimeMillis() >= hitTarget + 100){
			drivebase.disableAutoControl();
			//vision.setLEDState(Vision.LEDState.OFF);
			return CommandResult.COMPLETE;
		}else{
			return CommandResult.IN_PROGRESS;
		}
	}

	@Override
	public void done(final Robot robot) {
	}
}