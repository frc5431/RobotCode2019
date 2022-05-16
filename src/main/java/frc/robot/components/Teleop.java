package frc.robot.components;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.auto.Sequence;
import frc.robot.auto.SequenceType;
import frc.robot.components.Climber.ForkState;
import frc.robot.components.Intake.FingerState;
import frc.robot.components.Intake.JayState;
import frc.robot.util.ControlMode;
import frc.robot.util.Titan;

public class Teleop extends Titan.Component<Robot> {
	private Titan.Xbox driver;
	private Titan.LogitechExtreme3D operator;

	private Titan.Toggle fingers, jay, forks;

	private double powerMatrix[][] = {
        {  0.60,  0.70,  0.80,  0.90,   1.00,   1.00,  1.00,  1.00,  1.00 },
        {  0.50,  0.56,  0.63,  0.69,   0.75,   0.81,  0.88,  0.94,  1.00 },
        {  0.00,  0.13,  0.25,  0.38,   0.50,   0.63,  0.75,  0.88,  1.00 },
		{ -0.50, -0.31, -0.13,  0.06,   0.25,   0.44,  0.63,  0.81,  1.00 },
		
		{ -1.00, -0.75, -0.50, -0.25,   0.00,   0.25,  0.50,  0.75,  1.00 },
		
        { -1.00, -0.81, -0.63, -0.44,  -0.25,  -0.06,  0.13,  0.31,  0.50 },
        { -1.00, -0.88, -0.75, -0.63,  -0.50,  -0.38, -0.25, -0.13,  0.00 },
        { -1.00, -0.94, -0.88, -0.81,  -0.75,  -0.69, -0.63, -0.56, -0.50 },
        { -1.00, -1.00, -1.00, -1.00,  -1.00,  -0.90, -0.80, -0.70, -0.60 } };


	private double leftLast = 0, rightLast = 0;
	public Teleop() {
		driver = new Titan.Xbox(Constants.DRIVER_JOYSTICK_ID);
		driver.setDeadzone(Constants.DRIVER_JOYSTICK_DEADZONE);

		operator = new Titan.LogitechExtreme3D(Constants.OPERATOR_JOYSTICK_ID);
		operator.setDeadzone(Constants.OPERATOR_JOYSTICK_DEADZONE);

		fingers = new Titan.Toggle();
		jay = new Titan.Toggle();
		forks = new Titan.Toggle();
	}

	@Override
	public void init(final Robot robot) {

	}

	@Override
	public void periodic(final Robot robot) {
		final Climber climber = robot.getClimber();
		// System.out.printf(driver.getName());
		if (driver.getName().toUpperCase().contains("XBOX")) {
			final Drivebase drivebase = robot.getDrivebase();

			double left;
			double right;

			if (robot.getDashboard().getTankChooser()) {
				left = -driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y);
				right = -driver.getRawAxis(Titan.Xbox.Axis.RIGHT_Y);
			} else {

				double joy_x = 0.75*driver.getRawAxis(Titan.Xbox.Axis.RIGHT_X);
				double joy_y = -driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y);
				double leftEncoder = drivebase.leftEncoder.getRate(); 
				double rightEncoder = drivebase.rightEncoder.getRate();
				
				if(drivebase.rightEncoder.getRaw() < 0){
					
					rightEncoder*=-1; 
				}
				if(!drivebase.leftEncoder.getDirection() )
				{
					leftEncoder*=-1;
				}

				int px = (int) Math.round(joy_x * 4);
				int py = (int) Math.round(joy_y * 4);

				left = powerMatrix[4-py][4+px];
				right = powerMatrix[4-py][4-px];
				

				//left = -driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y)+driver.getRawAxis(Titan.Xbox.Axis.LEFT_X)*.5;
				//right = -driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y)-driver.getRawAxis(Titan.Xbox.Axis.LEFT_X)*.5;

				double zeroleft = left;
				double zeroright = right;

				boolean leftNeg = false;
				boolean rightNeg = false;
				if (left < 0) {
					leftNeg = true;
					left = Math.abs(left);
					leftLast = Math.abs(leftLast);
				}
				if (right < 0) {
					rightNeg = true;
					right = Math.abs(right);
					rightLast = Math.abs(rightLast);
				}


				double aaa = 0.05;
				if (leftLast+aaa < left) {
					left = leftLast + aaa;
				}
				if (rightLast+aaa < right) {
					right = rightLast + aaa;
				}

				if (leftNeg) {
					left = -left;
				}
				if (rightNeg) {
					right = -right;
				}



				//System.out.println(leftLast);
				//System.out.println(rightLast);

				leftLast = left;
				rightLast = right;

				if(zeroleft == 0) left = 0;
				if(zeroright == 0) right = 0;


				// System.out.println(DriverStation.getBatteryVoltage());
				// if(DriverStation.getBatteryVoltage() < 9) {
				// 	left = left / 2;
				// 	right = right / 2;
				// }


			}

			// xbox controllers have inverted controls
			if (drivebase.getControlMode() == ControlMode.MANUAL || left != 0.0 || right != 0.0) {
				drivebase.disableAllPID();

				drivebase.setControlMode(ControlMode.MANUAL);
				drivebase.drive(Math.pow(left, 2) * Math.signum(left), Math.pow(right, 2) * Math.signum(right));
				//drivebase.drive(left,right);
				}

			climber.climb(
					driver.getRawAxis(Titan.Xbox.Axis.TRIGGER_LEFT) - driver.getRawAxis(Titan.Xbox.Axis.TRIGGER_RIGHT));
		}

		if (operator.getName().equalsIgnoreCase("Logitech Extreme 3D")) {
			final Elevator elevator = robot.getElevator();
			final double elevPower = -operator.getRawAxis(Titan.LogitechExtreme3D.Axis.Y);
			if (elevPower != 0 || (elevPower == 0 && robot.getElevator().getControlMode() == ControlMode.MANUAL)) {
				elevator.setControlMode(ControlMode.MANUAL);
				elevator.elevate(elevPower);
			}

			final Intake intake = robot.getIntake();
			if (operator.getRawButton(Titan.LogitechExtreme3D.Button.TRIGGER)
					|| robot.getAuton().isSequencePressed(SequenceType.CARGO, Sequence.INTAKE)) {
				intake.setControlMode(ControlMode.MANUAL);
				intake.roll(Constants.INTAKE_ROLLER_SPEED);
			} else if (operator.getRawButton(Titan.LogitechExtreme3D.Button.TWO)
					|| robot.getAuton().isSequencePressed(SequenceType.CARGO, Sequence.OUTTAKE)) {
				intake.setControlMode(ControlMode.MANUAL);
				intake.roll(-Constants.INTAKE_ROLLER_SPEED);
			} else if (intake.getControlMode() == ControlMode.MANUAL) {
				intake.roll(0);
			}

			fingers.setState(intake.getFingerState() == FingerState.DEPLOYED);
			intake.finger(
					fingers.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.SIX)) ? FingerState.DEPLOYED
							: FingerState.RETRACTED);
			jay.setState(intake.getJayState() == JayState.DEPLOYED);
			intake.jay(jay.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.SEVEN)) ? JayState.DEPLOYED
					: JayState.RETRACTED);

			final Arm arm = robot.getArm();
			if (operator.getPOV() == 0) {
				arm.setControlMode(ControlMode.MANUAL);
				arm.pivot(Constants.ARM_PIVOT_UP_SPEED);
			} else if (operator.getPOV() == 180) {
				arm.setControlMode(ControlMode.MANUAL);
				arm.pivot(-Constants.ARM_PIVOT_DOWN_SPEED);
			} else if (arm.getControlMode() == ControlMode.MANUAL) {
				arm.pivot(0.0);
			}

			forks.setState(climber.getForkState() == ForkState.DEPLOYED);
			climber.fork(forks.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.TEN)) ? ForkState.DEPLOYED
					: ForkState.RETRACTED);
		}
	}

	@Override
	public void disabled(final Robot robot) {

	}

	public Titan.Xbox getDriver() {
		return driver;
	}

	public Titan.LogitechExtreme3D getOperator() {
		return operator;
	}
}