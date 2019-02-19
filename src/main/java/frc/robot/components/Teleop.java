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
        System.out.println(robot.getIntake().getHatchDistance() + ", " + robot.getArm().getWristPosition() + ", " + robot.getElevator().getEncoderPosition());
        final Drivebase drivebase = robot.getDrivebase();
        final double left = driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y);
        final double right = driver.getRawAxis(Titan.Xbox.Axis.RIGHT_Y);
        drivebase.drive(Math.pow(left, 2) * Math.signum(left), Math.pow(right, 2) * Math.signum(right));

        final Climber climber = robot.getClimber();
        climber.climb(driver.getRawAxis(Titan.Xbox.Axis.TRIGGER_LEFT) - driver.getRawAxis(Titan.Xbox.Axis.TRIGGER_RIGHT));
        //robot.getElevator().elevate(driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y));
        
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

        //robot.getClimber().climb(driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y));
        //robot.getElevator().elevate(driver.getRawAxis(Titan.Xbox.Axis.RIGHT_Y));
        //robot.getClimber().climb(driver.getRawButton(Titan.Xbox.Button.BUMPER_L) ? Constants.CLIMBER_SPEED : 0.0);
    
        // final Elevator elevator = robot.getElevator();
        // final boolean braked = operator.getRawButton(Titan.LogitechExtreme3D.Button.TRIGGER);
        // elevator.elevate(braked ? 0.0 : operator.getRawAxis(Titan.LogitechExtreme3D.Axis.Y));
        // elevator.setBrake(braked);
    
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
        
        // final Intake intake = robot.getIntake();
        // if(operator.getRawButton(Titan.LogitechExtreme3D.Button.FIVE)){
        //   intake.roll(Constants.INTAKE_ROLLER_SPEED);
        // }else if(operator.getRawButton(Titan.LogitechExtreme3D.Button.THREE)){
        //   intake.roll(-Constants.INTAKE_ROLLER_SPEED);
        // }else{
        //   intake.roll(0);
        // }
    
        // intake.actuateHatch(operator.getRawButton(Titan.LogitechExtreme3D.Button.TWO));
        
    }

    public Titan.Xbox getDriver(){
      return driver;
    }

    public Titan.LogitechExtreme3D getOperator(){
      return operator;
    }
}