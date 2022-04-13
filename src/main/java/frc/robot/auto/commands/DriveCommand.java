package frc.robot.auto.commands;

import frc.robot.util.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.AutoType;

public class DriveCommand extends CommandBase {
	private final double left, right;
	private final Timer timer;
	private final long seconds;

	public DriveCommand(final double left, final double right, final long seconds) {
		// name = "DriveCommand";

		this.left = left;
		this.right = right;

		this.timer = new Timer();
		this.seconds = seconds;

		// properties = String.format("Left: %f; Right: %f; Time: %d", left, right, time);
	}
	
	@Override
	public void initialize() {
		final Drivebase drivebase = Robot.getRobot().getDrivebase();
		drivebase.prepareForAutoControl(AutoType.COMMANDS);
		timer.reset();
		timer.start();
	}

	@Override
	public boolean isFinished() {
		final Drivebase drivebase = Robot.getRobot().getDrivebase();

		if(drivebase.getControlMode() == ControlMode.MANUAL){
			Robot.getRobot().getAuton().abort(Robot.getRobot());
			return true;
		}

		drivebase.drive(left, right);

		if(timer.hasElapsed(seconds)){
			drivebase.disableAutoControl();
			return true;
		}else{
			return false;
		}
	}

	@Override
	public void end(boolean interrupted) {
		timer.stop();
	}
}