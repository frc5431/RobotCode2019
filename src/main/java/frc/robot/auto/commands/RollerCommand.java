package frc.robot.auto.commands;

import frc.robot.util.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.components.Intake;

public class RollerCommand extends CommandBase {
    private final double power;
    private final long seconds;
	private final Timer timer;

	public RollerCommand(final double power, final long seconds) {
		// name = "RollerCommand";

        this.power = power;
		this.seconds = seconds;
		this.timer = new Timer();
		
		// properties = String.format("Power: %f; Time: %d", power, time);
	}
	
	@Override
	public void initialize() {
		Robot.getRobot().getIntake().setControlMode(ControlMode.AUTO);
		timer.reset();
		timer.start();
    }

	@Override
	public boolean isFinished() {
		Robot robot = Robot.getRobot();
		final Intake intake = robot.getIntake();
		if(seconds >= 0 && intake.getControlMode() == ControlMode.MANUAL){
			robot.getAuton().abort(robot);
			return true;
		}

    	intake.roll(power);
		
		if(seconds < 0){
			return true;
		}else if(timer.hasElapsed(seconds)) {
			intake.roll(0);
			return true;
		}

		return false;
	}

	@Override
	public void end(boolean interrupted) {
		if(seconds >= 0){
			Robot.getRobot().getIntake().roll(0);
		}
		timer.stop();
	}
}