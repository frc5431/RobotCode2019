package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.util.Component;
import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;
import frc.robot.components.Teleop;
import frc.robot.components.Auton;
import frc.robot.components.Dashboard;
import frc.robot.components.Vision;

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

  private Vision vision;

  private Dashboard dashboard;

  @Override
  public void robotInit() {
    //Titan.DEBUG = Constants.ROBOT_TYPE == Constants.Robot.PRACTICE;

    compressor = new Compressor(30);
    compressor.start();
    compressor.setClosedLoopControl(true);
    //compressor.stop();

    climber = new Climber();
    drivebase = new Drivebase();
    elevator = new Elevator();
    arm = new Arm();
    intake = new Intake();

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

  public List<Component> getComponents(){
    return List.of(teleop, auton, dashboard, vision, arm, climber, drivebase, elevator, intake);
  }
}
