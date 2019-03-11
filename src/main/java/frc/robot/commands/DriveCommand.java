package frc.robot.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;

public class DriveCommand extends Titan.Command<Robot> {
	private final double left, right;
	private final double leftDistance, rightDistance;

	public DriveCommand(final double left, final double right, final double leftDistance, final double rightDistance, final double battery) {
		name = "DriveCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		properties = "Left: " + leftDistance + " (" + left + "%); Right: " + rightDistance + " (" + right + "%);";
	}
	
	@Override
	public void init(final Robot robot) {
		robot.getDrivebase().setControlMode(ControlMode.AUTO);

		robot.getDrivebase().enableDistancePID();

		robot.getDrivebase().setDistancePIDTarget(leftDistance, rightDistance);
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getDrivebase().getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		robot.getDrivebase().drive(left, right);
		
		return CommandResult.COMPLETE;
	}

	@Override
	public void done(final Robot robot) {
	}
}