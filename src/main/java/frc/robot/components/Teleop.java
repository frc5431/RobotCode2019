package frc.robot.components;

import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;
import frc.robot.Titan;
import frc.robot.Constants;
import frc.robot.ControlMode;
import frc.robot.Robot;

public class Teleop{
    private Titan.Xbox driver;
    private Titan.LogitechExtreme3D operator;

    private Titan.Toggle wrist, fingers, hatch;

    public Teleop(){
        driver = new Titan.Xbox(Constants.DRIVER_JOYSTICK_ID);
        driver.setDeadzone(Constants.DRIVER_JOYSTICK_DEADZONE);

        operator = new Titan.LogitechExtreme3D(Constants.OPERATOR_JOYSTICK_ID);
        operator.setDeadzone(Constants.OPERATOR_JOYSTICK_DEADZONE);

        wrist = new Titan.Toggle();
        fingers = new Titan.Toggle();
        hatch = new Titan.Toggle();
    }

    public void periodic(final Robot robot){
        final Drivebase drivebase = robot.getDrivebase();
        // xbox controllers have inverted controls
        final double left = -driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y);
        final double right = -driver.getRawAxis(Titan.Xbox.Axis.RIGHT_Y);
        if(drivebase.getControlMode() == ControlMode.MANUAL || left != 0.0 || right != 0.0){
          drivebase.disableAllPID();

          drivebase.setControlMode(ControlMode.MANUAL);
          drivebase.drive(Math.pow(left, 2) * Math.signum(left), Math.pow(right, 2) * Math.signum(right));
        }

        final Climber climber = robot.getClimber();
        climber.climb(driver.getRawAxis(Titan.Xbox.Axis.TRIGGER_LEFT) - driver.getRawAxis(Titan.Xbox.Axis.TRIGGER_RIGHT));
        
        final Elevator elevator = robot.getElevator();
        final double elevPower = -operator.getRawAxis(Titan.LogitechExtreme3D.Axis.Y);
        if(elevPower != 0 || (elevPower == 0 && robot.getElevator().getControlMode() == ControlMode.MANUAL)){
          elevator.setControlMode(ControlMode.MANUAL);
          elevator.elevate(elevPower);
        }

        final Intake intake = robot.getIntake();
        if(operator.getRawButton(Titan.LogitechExtreme3D.Button.TRIGGER)){
          intake.roll(Constants.INTAKE_ROLLER_SPEED);
        }else if(operator.getRawButton(Titan.LogitechExtreme3D.Button.TWO)){
          intake.roll(-Constants.INTAKE_ROLLER_SPEED);
        }else{
          intake.roll(0);
        }

        hatch.setState(intake.isHatchOuttaking());
        intake.actuateHatch(hatch.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.FOUR)));
        fingers.setState(intake.isFingering());
        intake.finger(fingers.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.SIX)));

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

        wrist.setState(robot.getArm().isWristing());
        arm.wrist(wrist.isToggled(operator.getRawButton(Titan.LogitechExtreme3D.Button.SEVEN)));      
    }

    public Titan.Xbox getDriver(){
      return driver;
    }

    public Titan.LogitechExtreme3D getOperator(){
      return operator;
    }
}