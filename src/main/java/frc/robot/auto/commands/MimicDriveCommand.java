package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;

public class MimicDriveCommand extends Titan.Command<Robot> {
	private final double left, right;
	private final double leftDistance, rightDistance;
	private final double battery;
	
	public MimicDriveCommand(final double left, final double right, final double leftDistance, final double rightDistance, final double battery) {
		name = "MimicDriveCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.battery = battery;

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

		final double voltageCompensation = battery / RobotController.getBatteryVoltage();

		robot.getDrivebase().drive(left * voltageCompensation, right * voltageCompensation);
		
		return CommandResult.COMPLETE;
	}

	@Override
	public void done(final Robot robot) {
	}
}