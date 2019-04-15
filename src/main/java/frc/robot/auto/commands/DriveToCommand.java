package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.AutoType;

public class DriveToCommand extends Titan.Command<Robot> {
	private final double left, right, leftDistance, rightDistance;
	private final AutoType type;

	public DriveToCommand(final double leftDistance, final double rightDistance, final double left, final double right, final AutoType type) {
		name = "DriveToCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.type = type;

		properties = String.format("Left: %f (%f%%); Right: %f (%f%%);", leftDistance, left, rightDistance, right);
	}

	public DriveToCommand(final double leftDistance, final double rightDistance, final double left, final double right){
		this(leftDistance, rightDistance, left, right, AutoType.COMMANDS);
	}
	
	public DriveToCommand(final double distance, final double speed, final AutoType type){
		this(distance, distance, speed, speed, type);
	}

	public DriveToCommand(final double distance, final double speed){
		this(distance, speed, AutoType.COMMANDS);
	}

	@Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.prepareForAutoControl(AutoType.DRIVE_TO);
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		drivebase.drive(left, right);

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