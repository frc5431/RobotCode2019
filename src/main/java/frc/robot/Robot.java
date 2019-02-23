package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;
import frc.robot.components.Teleop;
import frc.robot.components.Auton;
import frc.robot.components.Dashboard;

public class Robot extends TimedRobot {
  public static enum Mode{
    DISABLED, AUTO, TELEOP, TEST
  }

  private Mode mode = Mode.DISABLED;

  private Compressor compressor;

  private Climber climber;
  private Drivebase drivebase;
  private Elevator elevator;
  private Arm arm;
  private Intake intake;

  private Teleop teleop;
  private Auton auton;

  private Dashboard dashboard;

  @Override
  public void robotInit() {
    compressor = new Compressor(30);
    compressor.setClosedLoopControl(true);
    //compressor.stop();

    //elevator brake is 0
    //arm brake is 7
    //wrist is 6
    //hatch left is 3
    //hatch right is 4
    //fingers is 5

    climber = new Climber();
    drivebase = new Drivebase();
    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();

    teleop = new Teleop();
    auton = new Auton();
  
    dashboard = new Dashboard();
  }

  @Override
  public void robotPeriodic() {
    dashboard.periodic(this);
  }
  
  @Override
  public void teleopInit() {
    mode = Mode.TELEOP;
    auton.init(this);
  }

  @Override
  public void teleopPeriodic() {
    //compressor.stop();

    teleop.periodic(this);
    auton.periodic(this);

    drivebase.periodic(this);
    elevator.periodic(this);
    intake.periodic(this);
    arm.periodic(this);
    climber.periodic(this);
  }

  // Since the autonomous period is the sandstorm, and it uses the same code as teleop, to not repeat code, we just call the teleop methods from autonomous.

  @Override
  public void autonomousInit() {
    mode = Mode.AUTO;
    auton.init(this);
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void testInit(){
    mode = Mode.TEST;
    auton.init(this);
  }

  @Override
  public void testPeriodic(){
    teleopPeriodic();
  }

  @Override
  public void disabledInit(){
    mode = Mode.DISABLED;
    auton.disabled(this);
  }

  public Mode getMode(){
    return mode;
  }

  public Teleop getTeleop(){
    return teleop;
  }

  public Auton getAuton(){
    return auton;
  }
  
  public Dashboard getDashboard(){
    return dashboard;
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
