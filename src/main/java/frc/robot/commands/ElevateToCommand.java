package frc.robot.commands;

import frc.robot.Titan;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.ControlMode;

public class ElevateToCommand extends Titan.Command<Robot>{
	private final double position, speed;
	private PIDController pid = null;

	private double speedOffset = 0.0;

	public ElevateToCommand(final double position, final double spd) {
        this.position = position;
        /* sped */
        this.speed = spd;

		name = "ElevateToCommand";
		properties = String.format("Position %.2f : Speed %.2f", position, speed);
	}

	@Override
	public CommandResult update(final Robot robot) {
		if(robot.getElevator().getControlMode() == ControlMode.MANUAL){
			return CommandResult.CLEAR_QUEUE;
		}

		if(pid == null){
			return CommandResult.RESTART_COMMAND;
		}

		if (Titan.approxEquals(robot.getElevator().getEncoderPosition(), position, 750)) {
			robot.getElevator().elevate(0.0);
			return CommandResult.COMPLETE;
		}

		if(robot.getElevator().getEncoderPosition() > position){
			robot.getElevator().elevate(-speed * Constants.AUTO_ELEVATOR_DOWN_MULTIPLIER /*+ speedOffset*/);
		}else{
			robot.getElevator().elevate(speed /*+ speedOffset*/);
		}

		return CommandResult.IN_PROGRESS;
	}

	@Override
	public void init(final Robot robot) {
		pid  = new PIDController(Constants.ELEVATOR_PID_P, Constants.ELEVATOR_PID_I, Constants.ELEVATOR_PID_D, new PIDSource(){
		
			@Override
			public void setPIDSourceType(final PIDSourceType pidSource) {
				//do nothing
			}
		
			@Override
			public double pidGet() {
				return robot.getElevator().getEncoderPosition();
			}
		
			@Override
			public PIDSourceType getPIDSourceType() {
				return PIDSourceType.kDisplacement;
			}
		}, new PIDOutput(){
		
			@Override
			public void pidWrite(final double output) {
				speedOffset = output;
			}
		});
		pid.enable();

		pid.setInputRange(0.0, 10000);
		pid.setOutputRange(-1.0, 1.0);
		pid.setSetpoint(position);

		robot.getElevator().setControlMode(ControlMode.AUTO);
	}

	@Override
	public void done(final Robot robot) {
		if(pid != null){
			pid.disable();
		}
	}
}