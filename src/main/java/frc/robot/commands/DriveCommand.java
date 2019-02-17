package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.Titan;
import frc.robot.Constants;

public class DriveCommand extends Titan.Command<Robot> {
    private final double distance, angle, speed;

	public DriveCommand(final double dis) {
		this(dis, 0.0);
	}

	public DriveCommand(final double dis, final double ang) {
		this(dis, ang, Constants.AUTO_ROBOT_DEFAULT_SPEED);
	}

	public DriveCommand(final double dis, final double ang, final double spd) {
		distance = dis;
		angle = ang;

		if (distance < 0)
			speed = -spd;
		else
			speed = spd;

		name = "DriveCommand";
		properties = String.format("Distance %.2f : Heading %.2f", distance, angle);
	}

	public double getDistance() {
		return distance;
	}

	@Override
	public CommandResult update(final Robot robot) {
		if (robot.getDrivebase().hasTravelled(distance)) {
			return CommandResult.COMPLETE;
		}
		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		robot.getDrivebase().reset();
		//robot.getDrivebase().drivePID(distance, angle, speed, DriveBase.TitanPIDSource.NAVX, Vision.TargetMode.Normal);
	}

	@Override
	public void done(final Robot robot) {
		//robot.getDrivebase().disableAllPID();
		robot.getDrivebase().drive(0.0, 0.0);
	}
}