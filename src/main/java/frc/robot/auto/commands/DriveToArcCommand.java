package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.ControlType;
import edu.wpi.first.wpilibj.RobotController;

public class DriveToArcCommand extends Titan.Command<Robot> {
	private final double left, right, leftDistance, rightDistance, angle, startRamp;

	private double startAngle;

	public DriveToArcCommand(final double left, final double right, final double leftDistance, final double rightDistance, final double angle, final double startRamp) {
		name = "DriveToArcCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.angle = angle;

		this.startRamp = startRamp;

		properties = "Left: " + leftDistance + " (" + left + "%); Right: " + rightDistance + " (" + right + "%); Angle: " + angle + "; Start ramp: " + startRamp;
	}

	public DriveToArcCommand(final double left, final double right, final double leftDistance, final double rightDistance, final double angle){
		this(left, right, leftDistance, rightDistance, angle, Constants.DRIVEBASE_ARC_DEFAULT_START_RAMP);
	}

	public DriveToArcCommand(final double dis, final double spd, final double ang, final double startRamp){
		this(spd, spd, dis, dis, ang, startRamp);
	}

	public DriveToArcCommand(final double dis, final double spd, final double ang){
		this(dis, spd, ang, Constants.DRIVEBASE_ARC_DEFAULT_START_RAMP);
	}

	public DriveToArcCommand(final double dis, final double spd){
		this(dis, spd, 0);
	}
	
	@Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.setControlMode(ControlMode.AUTO);
		drivebase.setControlType(ControlType.COMMANDS);

		startAngle = drivebase.getAngle();

		drivebase.resetEncoders();
		drivebase.disableAllPID();

		drivebase.enableAnglePID();
		drivebase.setAnglePIDTarget(startAngle);

		// drivebase.enableDistancePID();
		// drivebase.setDistancePIDTarget(leftDistance, rightDistance);
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		final double progress = getProgress(drivebase);

		if(getAverageDistance(drivebase) <= 30){
			drivebase.setAnglePIDTarget(angle);
		}else{
			//drivebase.setAnglePIDTarget(angle);
			drivebase.setAnglePIDTarget(Titan.lerp(startAngle, angle, progress));
		}

		final double voltageCompensation = 12.0 / RobotController.getBatteryVoltage();

		final double ramp;
		if(progress <= startRamp){
			ramp = 1.0;
		}else{
			ramp = Titan.lerp(1.0, 0.2, (progress - startRamp) / (1.0 - startRamp));
		}
		drivebase.drive(left * ramp * voltageCompensation, right * ramp * voltageCompensation);
		if(drivebase.hasTravelled(leftDistance, rightDistance)){
			drivebase.drive(0.0, 0.0);
			drivebase.disableAllPID();
			drivebase.setControlMode(ControlMode.MANUAL);
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

	public double getAverageDistance(final Drivebase drivebase){
		return (Math.abs(drivebase.getLeftError()) + Math.abs(drivebase.getRightError())) / 2;
	}

	public double getProgress(final Drivebase drivebase){
		return getAverageDistance(drivebase) / ((Math.abs(leftDistance) + Math.abs(rightDistance)) / 2);
	}

	@Override
	public void done(final Robot robot) {
	}
}