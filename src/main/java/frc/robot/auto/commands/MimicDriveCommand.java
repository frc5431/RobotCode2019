package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.AutoType;

public class MimicDriveCommand extends CommandBase {
	private final double left, right;
	private final double leftDistance, rightDistance;
	private final double angle;
	private final double battery;
	
	public MimicDriveCommand(final double left, final double right, final double leftDistance, final double rightDistance, final double angle, final double battery) {
		// name = "MimicDriveCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.angle = angle;

		this.battery = battery;

		// properties = String.format("Left: %f (%f%%); Right: %f (%f%%); Angle: %f; Battery: %f", leftDistance, left, rightDistance, right, angle, battery);
	}
	
	@Override
	public void initialize() {
		final Drivebase drivebase = Robot.getRobot().getDrivebase();
		drivebase.prepareForAutoControl(AutoType.MIMIC);

		drivebase.enableDistancePID();
		drivebase.setDistancePIDTarget(leftDistance, rightDistance);

		drivebase.enableAnglePID();
		drivebase.setAnglePIDTarget(angle);
	}

	@Override
	public boolean isFinished() {
		Robot robot = Robot.getRobot();
		final Drivebase drivebase = robot.getDrivebase();

		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return true;
		}

		final double voltageCompensation = battery / RobotController.getBatteryVoltage();

		drivebase.drive(left * voltageCompensation, right * voltageCompensation);
		
		return true;
	}
}