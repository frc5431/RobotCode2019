package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;

public class Robot extends TimedRobot {
  private static enum Mode{
    DISABLED, AUTO, TELEOP
  }

  private Mode mode = Mode.DISABLED;

  private Climber climber;
  private Drivebase drivebase;
  private Elevator elevator;
  private Arm arm;
  private Intake intake;

  private Teleop teleop;

  @Override
  public void robotInit() {
    teleop = new Teleop();

    climber = new Climber();
    drivebase = new Drivebase();
    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();
  }

  @Override
  public void robotPeriodic() {
  }

  
  @Override
  public void teleopInit() {
    mode = Mode.TELEOP;
  }

  @Override
  public void teleopPeriodic() {
    teleop.periodic(this);
  }

  // Since the autonomous period is the sandstorm, and it uses the same code as teleop, to not repeat code, we just call the teleop methods from autonomous.

  @Override
  public void autonomousInit() {
    mode = Mode.AUTO;
    teleopInit();
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void disabledInit(){
    mode = Mode.DISABLED;
  }

  public Mode getMode(){
    return mode;
  }

  public Teleop getTeleop(){
    return teleop;
  }
  
  public Arm getArm(){
    return arm;
  }

  public Climber getClimber(){
    return climber;
  }

  public Drivebase getDrivebase(){
     return drivebase;
  }

  public Elevator getElevator(){
    return elevator;
  }

  public Intake getIntake(){
    return intake;
  }
}
