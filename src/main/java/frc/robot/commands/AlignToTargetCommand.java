package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.util.Titan;
import frc.robot.util.ControlMode;
import frc.robot.components.Vision;
import frc.robot.components.Drivebase;

public class AlignToTargetCommand extends Titan.Command<Robot> {
	private long foundTarget = -1;

    @Override
	public void init(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();
		drivebase.setControlMode(ControlMode.AUTO);

		//drivebase.enableDistancePID();

		robot.getVision().setLEDState(Vision.LEDState.ON);

		name = "Move to vision target";
	}

	@Override
	public CommandResult update(final Robot robot) {
		final Drivebase drivebase = robot.getDrivebase();

		if(drivebase.getControlMode() == ControlMode.MANUAL){
			robot.getVision().setLEDState(Vision.LEDState.OFF);
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}
		
		final double[] distances = robot.getVision().getDistancesToTarget();
		System.out.println(distances[2]);
		//System.out.println(distances[0] +", " + distances[1] + ", " + distances[2]);
		if((distances[0] != 0 && distances[1] != 0 && Titan.approxEquals(distances[2], 0, 0.5)) || foundTarget > 0){
			drivebase.drive(0, 0);
			if(foundTarget == -1){
				foundTarget = System.currentTimeMillis();
			}else if(!Titan.approxEquals(distances[2], 0, 2)){
				foundTarget = -1;
			}else if(System.currentTimeMillis() > foundTarget + 200){
				robot.getVision().setLEDState(Vision.LEDState.OFF);
				return CommandResult.COMPLETE;
			} 
		}else{
			robot.getVision().setLEDState(Vision.LEDState.ON);

			//System.out.println("Waiting for angle: " + distances[2]);
			drivebase.drive(0.05 * Math.signum(distances[2]), -0.05 * Math.signum(distances[2]));
		}
        
        
		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void done(final Robot robot) {
		robot.getDrivebase().drive(0, 0);

		robot.getVision().setLEDState(Vision.LEDState.OFF);
	}
}