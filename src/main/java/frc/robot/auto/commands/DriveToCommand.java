package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.AutoType;

public class DriveToCommand extends Titan.Command<Robot> {
	private final double left, right, leftDistance, rightDistance;
	private double currentLeftRamp = 0.0, currentRightRamp = 0.0;

	public DriveToCommand(final double leftDistance, final double rightDistance, final double left, final double right) {
		name = "DriveToCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		properties = String.format("Left: %f (%f%%); Right: %f (%f%%);", leftDistance, left, rightDistance, right);
	}

	public DriveToCommand(final double distance, final double speed){
		this(distance, distance, speed, speed);
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

		if(Titan.approxEquals(currentLeftRamp, left, 0.02)){
			currentLeftRamp = left;
		}else if(currentLeftRamp < left){
			currentLeftRamp += 0.02;
		}else if(currentLeftRamp > left){
			currentLeftRamp -= 0.02;
		}

		if(Titan.approxEquals(currentRightRamp, right, 0.02)){
			currentRightRamp = right;
		}else if(currentRightRamp < right){
			currentRightRamp += 0.02;
		}else if(currentRightRamp > right){
			currentRightRamp -= 0.02;
		}

		drivebase.drive(currentLeftRamp, currentRightRamp);

		if(drivebase.hasTravelled(leftDistance, rightDistance)){
			drivebase.disableAutoControl();

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