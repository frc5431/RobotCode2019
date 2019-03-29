package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;

public class DriveCommand extends Titan.Command<Robot> {
	private final double left, right;
	private final long time;

	public DriveCommand(final double left, final double right, final long time) {
		name = "DriveCommand";

		this.left = left;
		this.right = right;

		this.time = time;

		properties = "Left: " + left + "; Right: " + right + "; Time: " + time;
	}
	
	@Override
	public void init(final Robot robot) {
		robot.getDrivebase().setControlMode(ControlMode.AUTO);
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getDrivebase().getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		robot.getDrivebase().drive(left, right);

		if(System.currentTimeMillis() > startTime + time){
			robot.getDrivebase().drive(0.0, 0.0);
			return CommandResult.COMPLETE;
		}else{
			return CommandResult.IN_PROGRESS;
		}
	}

	@Override
	public void done(final Robot robot) {
	}
}