package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.AutoType;

public class TurnCommand extends Titan.Command<Robot> {
	private final double speed, angle;

	private long hitTarget = -1;

	public TurnCommand(final double speed, final double angle) {
		name = "TurnCommand";

		this.speed = speed;

		this.angle = angle;

		properties = String.format("Speed: %f; Angle: %f", speed, angle);
	}

	public TurnCommand(final double angle){
		this(Constants.AUTO_TURN_MIN_SPEED, angle);
	}

	@Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.prepareForAutoControl(AutoType.POINT_TURN);

		drivebase.enableAnglePID();
		drivebase.setAnglePIDTarget(angle);

		// drivebase.enableDistancePID();
		// drivebase.setDistancePIDTarget(leftDistance, rightDistance);
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		final double currentAngle = drivebase.getAngle();

		drivebase.drive(speed * Math.signum(angle - currentAngle), -speed * Math.signum(angle - currentAngle));
		
		if(Titan.approxEquals(currentAngle, angle, Constants.DRIVEBASE_ANGLE_TOLERANCE)){
			if(hitTarget <= 0){
				hitTarget = System.currentTimeMillis();
			}
		}else{
			hitTarget = -1;
		}

		if(/*drivebase.hasTurned(angle)*/hitTarget > 0 && System.currentTimeMillis() >= hitTarget + 100){
			drivebase.disableAutoControl();

			return CommandResult.COMPLETE;
		}else{
			return CommandResult.IN_PROGRESS;
		}
	}

	@Override
	public void done(final Robot robot) {
	}
}