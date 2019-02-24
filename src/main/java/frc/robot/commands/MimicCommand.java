package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import frc.robot.Titan;
import frc.robot.Robot;
import frc.robot.MimicPropertyValue;
import frc.robot.ControlMode;

public class MimicCommand extends Titan.Command<Robot> {
    private final Titan.CommandQueue<Robot> commands = new Titan.CommandQueue<>();
	
	public MimicCommand(final Robot robot, final String file) {
		name = "MimicCommand";
        
        commands.clear();

		//Collect the mimic file
		final ArrayList<Titan.Mimic.Step<MimicPropertyValue>> steps = Titan.Mimic.load(file.toLowerCase(), MimicPropertyValue.class);
        for(final Titan.Mimic.Step<MimicPropertyValue> step : steps){
            final List<Titan.Command<Robot>> out = new ArrayList<>();
            if(step.getBoolean(MimicPropertyValue.HOME)){
                out.add(new Titan.ConsumerCommand<>((rob)->{
                    rob.getDrivebase().reset();
                }));
            }

            out.add(new DriveCommand(step.getDouble(MimicPropertyValue.LEFT_POWER), step.getDouble(MimicPropertyValue.RIGHT_POWER), step.getDouble(MimicPropertyValue.LEFT_DISTANCE), step.getDouble(MimicPropertyValue.RIGHT_DISTANCE), step.getDouble(MimicPropertyValue.BATTERY)));

 //robot.getDrivebase().drive(Math.pow(left, 2) * Math.signum(left), Math.pow(right, 2) * Math.signum(right));
                //final double power = (step.leftPower + step.rightPower) / 2.0;
                
				// robot.getDrivebase().updateStepResults(power, step.angle);
				// if(!robot.getDrivebase().hasTravelled(step.leftDistance) && !(Math.abs(power) < Constants.AUTO_PATHFINDING_OVERRIDE_NEXT_STEP_SPEED)) {
				// 	Titan.l("Mimic is falling behind!");
				// 	nextStep = false;
				// }

            if(out.size() == 1){
                commands.add(out.get(0));
            }else{
                final Titan.ParallelCommandGroup<Robot> group = new Titan.ParallelCommandGroup<>();
                for(final Titan.Command<Robot> com : out){
                    group.addCommand(robot, com);
                }
                commands.add(group);
            }
        }
        
        properties = String.format("Steps %d", commands.size());
	}
	
	@Override
	public void init(final Robot robot) {
        robot.getDrivebase().setHome();

        robot.getDrivebase().setControlMode(ControlMode.AUTO);
        
        commands.init(robot);
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

        if(commands.update(robot)){
            return CommandResult.IN_PROGRESS;
        }

        return CommandResult.COMPLETE;
	}

	@Override
	public void done(final Robot robot) {
        commands.done(robot);

		robot.getDrivebase().disableAllPID();
		robot.getDrivebase().drive(0.0, 0.0);
	}
}