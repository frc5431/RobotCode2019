package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.ControlType;

public class TurnCommand extends Titan.Command<Robot> {
	private final double speed, angle;

	public TurnCommand(final double speed, final double angle) {
		name = "TurnCommand";

		this.speed = speed;

		this.angle = angle;

		properties = "Speed: " + speed + "; Angle: " + angle;
	}

	@Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.setControlMode(ControlMode.AUTO);

		drivebase.setHome();

		drivebase.setControlType(ControlType.COMMANDS);

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

		drivebase.drive(speed * Math.signum(angle), -speed * Math.signum(angle));
		if(drivebase.hasTurned(angle)){
			drivebase.drive(0.0, 0.0);
			drivebase.disableAllPID();
			drivebase.setControlMode(ControlMode.MANUAL);
			return CommandResult.COMPLETE;
		}else{
			return CommandResult.IN_PROGRESS;
		}
	}

	@Override
	public void done(final Robot robot) {
	}
}