package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;
import frc.robot.components.Pneumatics;
import frc.robot.components.Teleop;
import frc.robot.components.Auton;
import frc.robot.components.Dashboard;
import frc.robot.components.Vision;

public class Robot extends TimedRobot {
  public static enum Mode{
    DISABLED, AUTO, TELEOP, TEST
  }

  private Mode mode = Mode.DISABLED;

  private Climber climber;
  private Drivebase drivebase;
  private Elevator elevator;
  private Arm arm;
  private Intake intake;

  private Pneumatics pneumatics;

  private Teleop teleop;
  private Auton auton;

  private Vision vision;

  private Dashboard dashboard;

  public static Robot instance = null;

  @Override
  public void robotInit() {

    //compressor.stop();

    climber = new Climber();
    drivebase = new Drivebase();
    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();

    pneumatics = new Pneumatics();

    teleop = new Teleop();
    auton = new Auton();

    vision = new Vision();
  
    dashboard = new Dashboard();
  }

  public static Robot getRobot() {
    if (instance == null) instance = new Robot();
    return instance;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  
  @Override
  public void teleopInit() {
    mode = Mode.TELEOP;
  }

  @Override
  public void teleopPeriodic() {
    //compressor.stop();
  }

  // Since the autonomous period is the sandstorm, and it uses the same code as teleop, to not repeat code, we just call the teleop methods from autonomous.

  @Override
  public void autonomousInit() {
    mode = Mode.AUTO;
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void testInit(){
    mode = Mode.TEST;
  }

  @Override
  public void testPeriodic(){
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

  public Vision getVision(){
    return vision;
  }
}
