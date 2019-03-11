package frc.robot.commands;

import frc.robot.util.Titan;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;

public class ElevateToCommand extends Titan.Command<Robot>{
	private final int targetPosition;
	private final double speed;
	private int startPosition = -1;

	public ElevateToCommand(final int position, final double spd) {
        this.targetPosition = position;
        /* sped */
        this.speed = spd;

		name = "ElevateToCommand";
		properties = "Position: " + position + " ; Speed: " + speed;
	}

	private double getElevatorSpeed(final Robot robot){
		final double error = Math.abs(targetPosition - robot.getElevator().getEncoderPosition());
		final double speedOffset = Constants.AUTO_ELEVATOR_ACCELERATION * Math.pow(Math.min(Constants.AUTO_ELEVATOR_ACCELERATION_MAX_ERROR, error) / Constants.AUTO_ELEVATOR_ACCELERATION_MAX_ERROR, 2);

		final double feedforward = robot.getElevator().isCarriageUp() ? Constants.AUTO_ELEVATOR_STAGE_2_FEEDFORWARD : 0;

		if(targetPosition <= 0 || robot.getElevator().getEncoderPosition() > targetPosition){
			return (-(speed + speedOffset) * Constants.AUTO_ELEVATOR_DOWN_MULTIPLIER)  + feedforward ;
		}else{
			return (speed + speedOffset) + feedforward ;
		}
	}

	private boolean isComplete(final Robot robot){
		return (targetPosition > 0 && Titan.approxEquals(robot.getElevator().getEncoderPosition(), targetPosition, Constants.ELEVATOR_POSITION_TOLERANCE)) || (targetPosition <= 0 && robot.getElevator().isCarriageDown());
	}

	private void runElevator(final Robot robot){
		if(Math.abs(targetPosition - startPosition) < 10000){
			// motion magic jerks in short ranges, so manually run it when the initial error is low
			robot.getElevator().elevate(getElevatorSpeed(robot));
		}else if(targetPosition <= 0 && robot.getElevator().getEncoderPosition() <= 1000 && !robot.getElevator().isCarriageDown()){
			// if trying to stow, and the elevator is almost there, just run it at a constant speed down as motion magic could undershoot
			robot.getElevator().elevate(-speed * Constants.AUTO_ELEVATOR_DOWN_MULTIPLIER);
		}else{
			// use motion magic
			robot.getElevator().elevateTo(targetPosition);
		}
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getElevator().getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		if (isComplete(robot)) {
			// stop the elevator
			robot.getElevator().elevate(0.0);
			return CommandResult.COMPLETE;
		}

		runElevator(robot);
		
		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		robot.getElevator().setControlMode(ControlMode.AUTO);

		startPosition = robot.getElevator().getEncoderPosition();

		if(!isComplete(robot)){
			runElevator(robot);
		}
	}

	@Override
	public void done(final Robot robot) {
	}
}