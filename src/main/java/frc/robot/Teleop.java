package frc.robot;

import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;
import frc.robot.Titan;

public class Teleop{
    //private Climber climber;
    private Drivebase drivebase;
    //private Elevator elevator;
    //private Arm arm;
    //private Intake intake;
  
    private Titan.Xbox driver;
    private Titan.LogitechExtreme3D operator;

    public Teleop(){
        //climber = new Climber();
        //drivebase = new Drivebase();
        //elevator = new Elevator();
        //arm = new Arm();
        //intake = new Intake();

        driver = new Titan.Xbox(Constants.DRIVER_JOYSTICK_ID);
        driver.setDeadzone(Constants.DRIVER_JOYSTICK_DEADZONE);

        operator = new Titan.LogitechExtreme3D(Constants.OPERATOR_JOYSTICK_ID);
        operator.setDeadzone(Constants.OPERATOR_JOYSTICK_DEADZONE);
    }

    public void periodic(){
        drivebase.drive(driver.getRawAxis(Titan.Xbox.Axis.LEFT_Y), driver.getRawAxis(Titan.Xbox.Axis.RIGHT_Y));
        /*
        climber.climb(driver.getRawButton(Titan.Xbox.Button.BUMPER_L) ? Constants.CLIMBER_SPEED : 0.0);
    
        final boolean braked = operator.getRawButton(Titan.LogitechExtreme3D.Button.TRIGGER);
        elevator.elevate(braked ? 0.0 : operator.getRawAxis(Titan.LogitechExtreme3D.Axis.Y));
        elevator.setBrake(braked);
    
        arm.pivot(operator.getPOV());
    
        if(operator.getPOV() == 0){
          arm.pivot(Constants.ARM_PIVOT_SPEED);
        }else if(operator.getPOV() == 180){
          arm.pivot(-Constants.ARM_PIVOT_SPEED);
        }else{
          arm.pivot(0.0);
        }
    
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
}