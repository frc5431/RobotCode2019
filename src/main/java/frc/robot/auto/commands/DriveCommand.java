package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.AutoType;

public class DriveCommand extends Titan.Command<Robot> {
	private final double left, right;
	private final long time;

	public DriveCommand(final double left, final double right, final long time) {
		name = "DriveCommand";

		this.left = left;
		this.right = right;

		this.time = time;

		properties = String.format("Left: %f; Right: %f; Time: %d", left, right, time);
	}
	
	@Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.prepareForAutoControl(AutoType.COMMANDS);
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();

		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		drivebase.drive(left, right);

		if(System.currentTimeMillis() > startTime + time){
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