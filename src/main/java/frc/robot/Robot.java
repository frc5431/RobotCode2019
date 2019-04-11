package frc.robot;

import java.util.List;

import frc.robot.util.Titan;
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

public class Robot extends Titan.Robot<Robot> {
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

  @Override
  public void robotInit() {
    Titan.DEBUG = false;
    //Titan.DEBUG = Constants.ROBOT_TYPE == Constants.Robot.PRACTICE;

    Titan.DEBUG = true;

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

  @Override
  public void robotPeriodic() {
    getComponents().forEach((com)->com.tick(this));
  }
  
  @Override
  public void teleopInit() {
    mode = Mode.TELEOP;
    getComponents().forEach((com)->com.init(this));
  }

  @Override
  public void teleopPeriodic() {
    //compressor.stop();

    getComponents().forEach((com)->com.periodic(this));
  }

  // Since the autonomous period is the sandstorm, and it uses the same code as teleop, to not repeat code, we just call the teleop methods from autonomous.

  @Override
  public void autonomousInit() {
    mode = Mode.AUTO;
    getComponents().forEach((com)->com.init(this));
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void testInit(){
    mode = Mode.TEST;
    getComponents().forEach((com)->com.init(this));
  }

  @Override
  public void testPeriodic(){
    teleopPeriodic();
  }

  @Override
  public void disabledInit(){
    mode = Mode.DISABLED;
    getComponents().forEach((com)->com.disabled(this));
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

  @Override
  public List<Titan.Component<Robot>> getComponents(){
    return List.of(teleop, auton, dashboard, vision, arm, climber, drivebase, elevator, intake, pneumatics);
  }
}