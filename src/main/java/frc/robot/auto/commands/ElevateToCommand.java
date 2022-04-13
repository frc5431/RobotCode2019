package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.components.Elevator;
import frc.robot.util.ControlMode;
import frc.team5431.titan.core.misc.Calc;

public class ElevateToCommand extends CommandBase{
	private final int targetPosition;

	public ElevateToCommand(final int position) {
        this.targetPosition = position;

		// name = "ElevateToCommand";
		// properties = String.format("Position: %d", position);
	}

	// private double getElevatorSpeed(final Robot robot){
	// 	final double error = Math.abs(targetPosition - robot.getElevator().getEncoderPosition());
	// 	final double speedOffset = Constants.AUTO_ELEVATOR_ACCELERATION * Math.pow(Math.min(Constants.AUTO_ELEVATOR_ACCELERATION_MAX_ERROR, error) / Constants.AUTO_ELEVATOR_ACCELERATION_MAX_ERROR, 2);

	// 	final double feedforward = robot.getElevator().isCarriageUp() ? Constants.AUTO_ELEVATOR_STAGE_2_FEEDFORWARD : 0;

	// 	if(targetPosition <= 0 || robot.getElevator().getEncoderPosition() > targetPosition){
	// 		return (-(speed + speedOffset) * Constants.AUTO_ELEVATOR_DOWN_MULTIPLIER)  + feedforward ;
	// 	}else{
	// 		return (speed + speedOffset) + feedforward ;
	// 	}
	// }

	private boolean isComplete(final Elevator elevator){
		return (targetPosition > 0 && Calc.approxEquals(elevator.getEncoderPosition(), targetPosition, Constants.ELEVATOR_POSITION_TOLERANCE)) || (targetPosition <= 0 && elevator.isCarriageDown());
	}

	private void runElevator(final Elevator elevator){
		if(targetPosition <= 0 && elevator.getEncoderPosition() <= 1000 && !elevator.isCarriageDown()){
			// if trying to stow, and the elevator is almost there, just run it at a constant speed down as motion magic could undershoot
			elevator.elevate(-Constants.AUTO_ELEVATOR_SPEED * Constants.AUTO_ELEVATOR_DOWN_MULTIPLIER);
		}else {
			// use motion magic
			elevator.elevateTo(Math.max(750, targetPosition));
		}
	}

	@Override
	public boolean isFinished() {
		Robot robot = Robot.getRobot();
		final Elevator elevator = robot.getElevator();
		if(elevator.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return true;
		}

		if (isComplete(elevator)) {
			// stop the elevator
			elevator.elevate(0.0);
			elevator.setControlMode(ControlMode.MANUAL);
			return true;
		}

		runElevator(elevator);
		
		return false;
	}

	@Override
	public void initialize() {
		final Elevator elevator = Robot.getRobot().getElevator();
		elevator.setControlMode(ControlMode.AUTO);

		if(!isComplete(elevator)){
			runElevator(elevator);
		}
	}
}