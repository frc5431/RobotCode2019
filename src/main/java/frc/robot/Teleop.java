package frc.robot;

import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;
import frc.robot.Titan;

public class Teleop{
    private Titan.Xbox driver;
    private Titan.LogitechExtreme3D operator;

    public Teleop(){
        driver = new Titan.Xbox(Constants.DRIVER_JOYSTICK_ID);
        driver.setDeadzone(Constants.DRIVER_JOYSTICK_DEADZONE);

        //operator = new Titan.LogitechExtreme3D(Constants.OPERATOR_JOYSTICK_ID);
        //operator.setDeadzone(Constants.OPERATOR_JOYSTICK_DEADZONE);
    }

    public void periodic(final Robot robot){
        //System.out.println(elevator.getEncoderPosition());
        //drivebase.drive(driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y), driver.getRawAxis(Titan.Xbox.Axis.RIGHT_Y));
        //robot.getClimber().climb(driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y));
        //robot.getClimber().climb(driver.getRawButton(Titan.Xbox.Button.BUMPER_L) ? Constants.CLIMBER_SPEED : 0.0);
    /*
        final Elevator elevator = robot.getElevator();
        final boolean braked = operator.getRawButton(Titan.LogitechExtreme3D.Button.TRIGGER);
        elevator.elevate(braked ? 0.0 : operator.getRawAxis(Titan.LogitechExtreme3D.Axis.Y));
        elevator.setBrake(braked);
    
        final Arm arm = robot.getArm();
        if(operator.getPOV() == 0){
          arm.pivot(Constants.ARM_PIVOT_SPEED);
        }else if(operator.getPOV() == 180){
          arm.pivot(-Constants.ARM_PIVOT_SPEED);
        }else{
          arm.pivot(0.0);
        }
    
        final Intake intake = robot.getIntake();
        if(operator.getRawButton(Titan.LogitechExtreme3D.Button.FIVE)){
          intake.roll(Constants.INTAKE_ROLLER_SPEED);
        }else if(operator.getRawButton(Titan.LogitechExtreme3D.Button.THREE)){
          intake.roll(-Constants.INTAKE_ROLLER_SPEED);
        }else{
          intake.roll(0);
        }
    
        intake.actuateHatch(operator.getRawButton(Titan.LogitechExtreme3D.Button.TWO));
        */
    }

    public Titan.Xbox getDriver(){
      return driver;
    }

    public Titan.LogitechExtreme3D getOperator(){
      return operator;
    }
}