package frc.robot.components;

import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;
import frc.robot.components.Climber.ForkState;
import frc.robot.auto.Sequence;
import frc.robot.auto.SequenceType;
import frc.robot.components.Intake.FingerState;
import frc.robot.components.Intake.JayState;
import frc.robot.util.Titan;
import frc.robot.Constants;
import frc.robot.util.ControlMode;
import frc.robot.Robot;

public class Teleop extends Titan.Component<Robot>{
		private Titan.Xbox driver;
		private Titan.LogitechExtreme3D operator;

		private Titan.Toggle fingers, jay, forks;

		public Teleop(){
			driver = new Titan.Xbox(Constants.DRIVER_JOYSTICK_ID);
			driver.setDeadzone(Constants.DRIVER_JOYSTICK_DEADZONE);

			operator = new Titan.LogitechExtreme3D(Constants.OPERATOR_JOYSTICK_ID);
			operator.setDeadzone(Constants.OPERATOR_JOYSTICK_DEADZONE);

			fingers = new Titan.Toggle();
			jay = new Titan.Toggle();
			forks = new Titan.Toggle();
		}

		@Override
		public void init(final Robot robot){
				
		}

		@Override
		public void periodic(final Robot robot){
      final Climber climber = robot.getClimber();
			if(driver.getName().equalsIgnoreCase("XBOX 360 For Windows (Controller)")){
				final Drivebase drivebase = robot.getDrivebase();
				// xbox controllers have inverted controls
				final double left = -driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y);
				final double right = -driver.getRawAxis(Titan.Xbox.Axis.RIGHT_Y);
				if(drivebase.getControlMode() == ControlMode.MANUAL || left != 0.0 || right != 0.0){
					drivebase.disableAllPID();

					drivebase.setControlMode(ControlMode.MANUAL);
					drivebase.drive(Math.pow(left, 2) * Math.signum(left), Math.pow(right, 2) * Math.signum(right));
				}

        climber.climb(driver.getRawAxis(Titan.Xbox.Axis.TRIGGER_LEFT) - driver.getRawAxis(Titan.Xbox.Axis.TRIGGER_RIGHT));
			}

			if(operator.getName().equalsIgnoreCase("Logitech Extreme 3D")){
				final Elevator elevator = robot.getElevator();
				final double elevPower = -operator.getRawAxis(Titan.LogitechExtreme3D.Axis.Y);
				if(elevPower != 0 || (elevPower == 0 && robot.getElevator().getControlMode() == ControlMode.MANUAL)){
					elevator.setControlMode(ControlMode.MANUAL);
					elevator.elevate(elevPower);
				}

				final Intake intake = robot.getIntake();
				if(operator.getRawButton(Titan.LogitechExtreme3D.Button.TRIGGER) || robot.getAuton().isSequencePressed(SequenceType.CARGO, Sequence.INTAKE)){
					intake.setControlMode(ControlMode.MANUAL);
					intake.roll(Constants.INTAKE_ROLLER_SPEED);
				}else if(operator.getRawButton(Titan.LogitechExtreme3D.Button.TWO) || robot.getAuton().isSequencePressed(SequenceType.CARGO, Sequence.OUTTAKE)){
					intake.setControlMode(ControlMode.MANUAL);
					intake.roll(-Constants.INTAKE_ROLLER_SPEED);
				}else if(intake.getControlMode() == ControlMode.MANUAL){
					intake.roll(0);
				}

				fingers.setState(intake.getFingerState() == FingerState.DEPLOYED);
				intake.finger(fingers.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.SIX)) ? FingerState.DEPLOYED : FingerState.RETRACTED);
				jay.setState(intake.getJayState() == JayState.DEPLOYED);
				intake.jay(jay.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.SEVEN)) ? JayState.DEPLOYED : JayState.RETRACTED);

				final Arm arm = robot.getArm();
				if(operator.getPOV() == 0){
					arm.setControlMode(ControlMode.MANUAL);
					arm.pivot(Constants.ARM_PIVOT_UP_SPEED);
				}else if(operator.getPOV() == 180){
					arm.setControlMode(ControlMode.MANUAL);
					arm.pivot(-Constants.ARM_PIVOT_DOWN_SPEED);
				}else if(arm.getControlMode() == ControlMode.MANUAL){
					arm.pivot(0.0);
        }
        
        forks.setState(climber.getForkState() == ForkState.DEPLOYED);
				climber.fork(forks.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.TEN)) ? ForkState.DEPLOYED : ForkState.RETRACTED);
			}
		}

		@Override
		public void disabled(final Robot robot){
				
		}

		public Titan.Xbox getDriver(){
			return driver;
		}

		public Titan.LogitechExtreme3D getOperator(){
			return operator;
		}
}