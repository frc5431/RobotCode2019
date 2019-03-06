package frc.robot.commands;

import frc.robot.util.Titan;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.ControlMode;

public class ElevateToCommand extends Titan.Command<Robot>{
	private final double position, speed;
	//private PIDController pid = null;

	public ElevateToCommand(final double position, final double spd) {
        this.position = position;
        /* sped */
        this.speed = spd;

		name = "ElevateToCommand";
		properties = String.format("Position %.2f : Speed %.2f", position, speed);
	}

	private double getElevatorSpeed(final Robot robot){
		final double error = Math.abs(position - robot.getElevator().getEncoderPosition());
		final double speedOffset = Constants.AUTO_ELEVATOR_ACCELERATION * (Math.min(Constants.AUTO_ELEVATOR_ACCELERATION_MAX_ERROR, error) / Constants.AUTO_ELEVATOR_ACCELERATION_MAX_ERROR);

		//final double stallVoltage = robot.getElevator().isCarriageUp() ? Constants.AUTO_ELEVATOR_STAGE_2_STALL : Constants.AUTO_ELEVATOR_STAGE_1_STALL;

		if(robot.getElevator().getEncoderPosition() > position){
			return (-(speed + speedOffset) /* + stallVoltage */ * Constants.AUTO_ELEVATOR_DOWN_MULTIPLIER /*+ speedOffset*/);
		}else{
			return (speed + speedOffset/* + stallVoltage */);
		}
	}

	private boolean isComplete(final Robot robot){
		return (position > 0 && Titan.approxEquals(robot.getElevator().getEncoderPosition(), position, Constants.ELEVATOR_POSITION_TOLERANCE)) || (position <= 0 && robot.getElevator().getEncoderPosition() <= Constants.ELEVATOR_BOTTOM_LIMIT);
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getElevator().getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return CommandResult.CLEAR_QUEUE;
		}

		// if(pid == null){
		// 	return CommandResult.RESTART_COMMAND;
		// }

		if (isComplete(robot)) {
			robot.getElevator().elevate(0.0);
			return CommandResult.COMPLETE;
		}

		robot.getElevator().elevate(getElevatorSpeed(robot));

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		// pid  = new PIDController(Constants.ELEVATOR_PID_P, Constants.ELEVATOR_PID_I, Constants.ELEVATOR_PID_D, new PIDSource(){
		
		// 	@Override
		// 	public void setPIDSourceType(final PIDSourceType pidSource) {
		// 		//do nothing
		// 	}
		
		// 	@Override
		// 	public double pidGet() {
		// 		return robot.getElevator().getEncoderPosition();
		// 	}
		
		// 	@Override
		// 	public PIDSourceType getPIDSourceType() {
		// 		return PIDSourceType.kDisplacement;
		// 	}
		// }, new PIDOutput(){
		
		// 	@Override
		// 	public void pidWrite(final double output) {
		// 		speedOffset = output;
		// 	}
		// });
		// pid.enable();

		// pid.setInputRange(0.0, 10000);
		// pid.setOutputRange(-1.0, 1.0);
		// pid.setSetpoint(position);

		robot.getElevator().setControlMode(ControlMode.AUTO);

		if(!isComplete(robot)){
			robot.getElevator().elevate(getElevatorSpeed(robot));
		}
	}

	@Override
	public void done(final Robot robot) {
		// if(pid != null){
		// 	pid.disable();
		// }
	}
}