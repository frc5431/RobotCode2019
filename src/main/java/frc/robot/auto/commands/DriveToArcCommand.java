package frc.robot.auto.commands;

import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.components.Drivebase;
import frc.robot.components.Drivebase.ControlType;
import edu.wpi.first.wpilibj.RobotController;

public class DriveToArcCommand extends Titan.Command<Robot> {
	private final double left, right, leftDistance, rightDistance, angle;

	public DriveToArcCommand(final double left, final double right, final double leftDistance, final double rightDistance, final double angle) {
		name = "DriveToArcCommand";

		this.left = left;
		this.right = right;

		this.leftDistance = leftDistance;
		this.rightDistance = rightDistance;

		this.angle = angle;

		properties = "Left: " + leftDistance + " (" + left + "%); Right: " + rightDistance + " (" + right + "%); Angle: " + angle;
	}

	public DriveToArcCommand(final double dis, final double spd, final double ang){
		this(spd, spd, dis, dis, ang);
	}

	public DriveToArcCommand(final double dis, final double spd){
		this(dis, spd, 0);
	}
	
	@Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.setControlMode(ControlMode.AUTO);
		drivebase.setControlType(ControlType.COMMANDS);

		drivebase.setHome();

		drivebase.enableAnglePID();
		drivebase.setAnglePIDTarget(0);

		// drivebase.enableDistancePID(PIDType.STANDARD);
		// drivebase.setDistancePIDTarget(leftDistance, rightDistance);
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		final double dis = (Math.abs(drivebase.getLeftError()) + Math.abs(drivebase.getRightError())) / 2;
		final double angleP = dis / ((Math.abs(leftDistance) + Math.abs(rightDistance)) / 2);

		if((Math.abs(leftDistance) + Math.abs(rightDistance)) / 2 <= 30){
			drivebase.setAnglePIDTarget(angle);
		}else{
			drivebase.setAnglePIDTarget(angle * angleP);
		}

		final double voltageCompensation = 12.0 / RobotController.getBatteryVoltage();

		drivebase.drive(left * (1.0 - (angleP * 0.3)) * voltageCompensation, right * (1.0 - (angleP * 0.3)) * voltageCompensation);
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

	@Override
	public void done(final Robot robot) {
	}
}