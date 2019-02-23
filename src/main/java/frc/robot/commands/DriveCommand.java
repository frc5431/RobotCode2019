package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotController;

import frc.robot.Titan;
import frc.robot.Constants;
import frc.robot.ControlMode;
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
		// double power = 0.0;
		// double angle = 0.0;
		// try {
		// 	final Titan.Mimic.Step<MimicPropertyValue> step = steps.get(0);
		// 	power = (step.leftPower + step.rightPower) / 2.0;
		// 	angle = step.angle;
		// } catch (Throwable ignored) {}
		//robot.getDrivebase().driveAtAnglePID(power, angle, TitanPIDSource.NAVX_MIMIC, Vision.TargetMode.Normal);
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getDrivebase().getControlMode() == ControlMode.MANUAL){
			return CommandResult.CLEAR_QUEUE;
		}

		//final double batteryMultiplier = battery / RobotController.getBatteryVoltage();
		//System.out.println(left + ", " + right);
		robot.getDrivebase().drive((Math.pow(left, 2) * Math.signum(left)), (Math.pow(right, 2) * Math.signum(right)));
		//final double power = (step.leftPower + step.rightPower) / 2.0;
		
		// robot.getDrivebase().updateStepResults(power, step.angle);
		if(robot.getDrivebase().isAtDistancePIDTarget() || /*robot.getDrivebase().hasTravelled(robot.getDrivebase().getLeftDistance(), robot.getDrivebase().getRightDistance()) ||*/ System.currentTimeMillis() > startTime + 100) {
			return CommandResult.COMPLETE;
		}

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void done(final Robot robot) {
		//robot.getDrivebase().disableAllPID();
		//robot.getDrivebase().disableAllPID();
		//robot.getDrivebase().drive(0.0, 0.0);
	}
}