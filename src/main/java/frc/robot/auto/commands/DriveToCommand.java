package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;

public class DriveToCommand extends Titan.Command<Robot> {
	private final double left, right, leftDistance, rightDistance;

	public DriveToCommand(final double left, final double right, final double leftDistance, final double rightDistance) {
		name = "DriveToCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		properties = "Left: " + left + "; Right: " + right;
	}
	
	@Override
	public void init(final Robot robot) {
		robot.getDrivebase().setControlMode(ControlMode.AUTO);

		robot.getDrivebase().setHome();
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getDrivebase().getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		final double percent = Math.min(1.0, ((double)System.currentTimeMillis() - (double)startTime) / 500.0);
		robot.getDrivebase().drive(left * percent, right * percent);

		if(robot.getDrivebase().hasTravelled(leftDistance, rightDistance)){
			robot.getDrivebase().drive(0.0, 0.0);
			return CommandResult.COMPLETE;
		}else{
			return CommandResult.IN_PROGRESS;
		}
	}

	public double getLeftDistance(){
		return leftDistance;
	}

	public double getRightDistance(){
		return rightDistance;
	}

	@Override
	public void done(final Robot robot) {
	}
}