package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Titan;
import frc.robot.Robot;
import frc.robot.MimicPropertyValue;
import frc.robot.ControlMode;
import frc.robot.Constants;

public class MimicCommand extends Titan.Command<Robot> {
    private int currentStep = 0;
	private int skippedSteps = 0;
	private final ArrayList<Titan.Mimic.Step<MimicPropertyValue>> steps;
	
	public MimicCommand(final String file) {
		name = "MimicCommand";
		
		//Collect the mimic file
		steps = Titan.Mimic.load(file.toLowerCase(), MimicPropertyValue.class);
		properties = String.format("Steps %d", steps.size());
	}
	
	@Override
	public void init(final Robot robot) {
		robot.getDrivebase().setHome();
		double leftPower = 0.0, rightPower = 0.0;
		double angle = 0.0;
		try {
			final Titan.Mimic.Step<MimicPropertyValue> step = steps.get(0);
            leftPower = step.getDouble(MimicPropertyValue.LEFT_POWER);
            rightPower = step.getDouble(MimicPropertyValue.RIGHT_POWER);
			angle = step.getDouble(MimicPropertyValue.ANGLE);
		} catch (Throwable ignored) {}
		//robot.getDrivebase().driveAtAnglePID(leftPower, rightPower, angle);
	}

	@Override
	public CommandResult update(final Robot robot) {
		boolean nextStep = true;
		try {
			final Titan.Mimic.Step<MimicPropertyValue> step = steps.get(currentStep);
			if(step.getBoolean(MimicPropertyValue.HOME)) {
				robot.getDrivebase().reset(); //Do not call setHome because that disables PID
			} else {
                //robot.getDriveBase().drive(step.leftPower, step.rightPower);		
                final double leftPower = step.getDouble(MimicPropertyValue.LEFT_POWER);
                final double rightPower = step.getDouble(MimicPropertyValue.RIGHT_POWER);
				//robot.getDrivebase().updateStepResults(leftPower, rightPower, step.getDouble(MimicPropertyValue.ANGLE));
				if(!robot.getDrivebase().hasTravelled(step.getDouble(MimicPropertyValue.LEFT_DISTANCE), step.getDouble(MimicPropertyValue.RIGHT_DISTANCE)) && !(Math.abs(leftPower) < Constants.AUTO_PATHFINDING_OVERRIDE_NEXT_STEP_SPEED) && !(Math.abs(rightPower) < Constants.AUTO_PATHFINDING_OVERRIDE_NEXT_STEP_SPEED)) {
					Titan.l("Mimic is falling behind!");
					nextStep = false;
				}
			}
		} catch (final IndexOutOfBoundsException e) {}
		if(nextStep || skippedSteps > 5) {
			if(((currentStep++) + 1) > steps.size()) return CommandResult.COMPLETE;
			skippedSteps = 0;
		} else {
			skippedSteps++;
		}
		return CommandResult.IN_PROGRESS;
		/*try {
			Stepper step = steps.get(currentStep);
			if(!step.isDrive && !step.isTurn) {
				robot.getDriveBase().setHome();
			} else if(step.isDrive && !wasDrive) {
				robot.getDriveBase().driveAtAnglePID(step.drivePower, step.angle, TitanPIDSource.NAVX, Vision.TargetMode.Normal);
				nextStep = false;
			} else if(step.isDrive  && wasDrive) {
				robot.getDriveBase().updateStepResults(step.drivePower, step.angle);
				if(!robot.getDriveBase().hasTravelled(step.distance) && !(Math.abs(step.drivePower) < Constants.AUTO_PATHFINDING_OVERRIDE_NEXT_STEP_SPEED)) {
					Titan.l("Pathfinding is falling behind!");
					nextStep = false;
				}
			} else if(step.isTurn && !wasTurn) {
				robot.getDriveBase().turnPID(step.angle, TitanPIDSource.NAVX, Vision.TargetMode.Normal);
				nextStep = false;
			} else if(step.isTurn && wasTurn) {
				robot.getDriveBase().updateStepResults(step.drivePower, step.angle);
				if(!robot.getDriveBase().hasTurned(step.angle) && !(Math.abs(step.angle) < Constants.AUTO_PATHFINDING_OVERRIDE_NEXT_STEP_TURN)) {
					Titan.l("Pathfinding is falling behind!");
					nextStep = false;
				}
			} else {
				Titan.e("Invalid step command!");
			}
			
			wasDrive = step.isDrive;
			wasTurn = step.isTurn;
		} catch (IndexOutOfBoundsException e) {}
		if(nextStep) if(((currentStep++) + 1) > steps.size()) return StepResult.COMPLETE;*/
	}

	@Override
	public void done(final Robot robot) {
		robot.getDrivebase().disableAllPID();
		robot.getDrivebase().drive(0.0, 0.0);
}
}