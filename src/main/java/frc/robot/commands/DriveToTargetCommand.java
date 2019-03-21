package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.components.Vision;
import frc.robot.components.Drivebase;

public class DriveToTargetCommand extends Titan.Command<Robot> {
	private double startLeft, startRight, targetLeft = 0, targetRight = 0;
	private double averageError = 0;
	private double lastErrorChange = -1;

    @Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.setControlMode(ControlMode.AUTO);

		lastErrorChange = System.currentTimeMillis();

		//drivebase.enableDistancePID();

		name = "Drive to vision target";
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();

		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}
		
		final double[] distances = robot.getVision().getDistancesToTarget();
		if(targetLeft == 0 || targetRight == 0){
			startLeft = drivebase.getLeftDistance();
			startRight = drivebase.getRightDistance();

			targetLeft = distances[0];
			targetRight = distances[1];

			averageError = getAverageError(drivebase);
			lastErrorChange = System.currentTimeMillis();
		}else{
			//System.out.println(targetLeft + ", " + targetRight);
			// drivebase.enableDistancePID();

			// drivebase.setDistancePIDTarget(distances);

			final double newAverageError = getAverageError(drivebase);
			if(averageError != newAverageError){
				lastErrorChange = System.currentTimeMillis();
			}
			averageError = newAverageError;

			System.out.println(targetLeft + ", " + targetRight);

			final boolean reachedLeft = drivebase.getLeftDistance() > startLeft + targetLeft;
			final boolean reachedRight = drivebase.getRightDistance() > startRight + targetRight;

			//System.out.println("Reached: " + reachedLeft + ", " + reachedRight);

			drivebase.drive(reachedLeft ? 0.2 : 0.2, reachedRight ? 0.2 : 0.2);

			if(System.currentTimeMillis() > lastErrorChange + 1000 || (reachedLeft || reachedRight)/* || System.currentTimeMillis() > startTime + 1000*/){
				drivebase.drive(0, 0);
				return CommandResult.COMPLETE;
			}
		}
        
		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void done(final Robot robot) {
		robot.getDrivebase().drive(0, 0);

		robot.getVision().setLEDState(Vision.LEDState.OFF);
	}

	private double getLeftError(final Drivebase drivebase){
		return drivebase.getLeftDistance() - (startLeft + targetLeft);
	}

	private double getRightError(final Drivebase drivebase){
		return drivebase.getRightDistance() - (startRight + targetRight);
	}

	private double getAverageError(final Drivebase drivebase){
		return (getLeftError(drivebase) + getRightError(drivebase)) / 2.0;
	}
}