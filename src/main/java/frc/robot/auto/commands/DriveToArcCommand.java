package frc.robot.auto.commands;

import frc.robot.util.ControlMode;
import frc.team5431.titan.core.misc.Calc;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.AutoType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToArcCommand extends CommandBase {
	private final double left, right, leftDistance, rightDistance, angle, startRamp;

	private double startAngle;

	public DriveToArcCommand(final double leftDistance, final double rightDistance, final double left, final double right, final double angle, final double startRamp) {
		// name = "DriveToArcCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.angle = angle;

		this.startRamp = startRamp;

		// properties = String.format("Left: %f (%f%%); Right: %f (%f%%); Angle: %f; Start ramp: %f", leftDistance, left, rightDistance, right, angle, startRamp);
	}

	public DriveToArcCommand(final double leftDistance, final double rightDistance, final double left, final double right, final double angle){
		this(leftDistance, rightDistance, left, right,  angle, Constants.DRIVEBASE_ARC_DEFAULT_START_RAMP);
	}

	public DriveToArcCommand(final double dis, final double spd, final double ang, final double startRamp){
		this(dis, dis, spd, spd, ang, startRamp);
	}

	public DriveToArcCommand(final double dis, final double spd, final double ang){
		this(dis, spd, ang, Constants.DRIVEBASE_ARC_DEFAULT_START_RAMP);
	}

	public DriveToArcCommand(final double dis, final double spd){
		this(dis, spd, 0);
	}
	
	@Override
	public void initialize() {
		final Drivebase drivebase = Robot.getRobot().getDrivebase();
		drivebase.prepareForAutoControl(AutoType.COMMANDS);

		startAngle = drivebase.getAngle();

		drivebase.enableAnglePID();
		drivebase.setAnglePIDTarget(startAngle);

		// drivebase.enableDistancePID();
		// drivebase.setDistancePIDTarget(leftDistance, rightDistance);
	}

	@Override
	public boolean isFinished() {
		Robot robot = Robot.getRobot();
		final Drivebase drivebase = robot.getDrivebase();
		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return true;
		}

		final double progress = getProgress(drivebase);

		if(getAverageTarget() <= 30){
			drivebase.setAnglePIDTarget(angle);
		}else{
			//drivebase.setAnglePIDTarget(angle);
			drivebase.setAnglePIDTarget(angle);
			//drivebase.setAnglePIDTarget(Titan.lerp(startAngle, angle, progress));
		}

		final double voltageCompensation = 12.0 / RobotController.getBatteryVoltage();

		final double ramp;
		if(progress < 0.2){
			ramp = Calc.lerp(0.2, 1.0, progress / 0.2);
		}else if(progress <= startRamp){
			ramp = 1.0;
		}else{
			ramp = Calc.lerp(1.0, 0.2, (progress - startRamp) / (1.0 - startRamp));
		}

		drivebase.drive(left * ramp * voltageCompensation, right * ramp * voltageCompensation);
		if(drivebase.hasTravelled(leftDistance, rightDistance)){
			drivebase.disableAutoControl();

			return true;
		}else{
			return false;
		}
	}

	public double getLeftDistance(){
		return leftDistance;
	}

	public double getRightDistance(){
		return rightDistance;
	}

	public double getAverageDistance(final Drivebase drivebase){
		return (Math.abs(drivebase.getLeftDistance()) + Math.abs(drivebase.getRightDistance())) / 2;
	}

	public double getAverageTarget(){
		return ((Math.abs(leftDistance) + Math.abs(rightDistance)) / 2);
	}

	public double getProgress(final Drivebase drivebase){
		return Math.max(0.0, Math.min(1.0, getAverageDistance(drivebase) / getAverageTarget()));
	}
}