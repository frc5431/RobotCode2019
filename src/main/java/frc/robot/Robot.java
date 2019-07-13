/* This file is used to define the components of the robot like the climber,elevator,arm,etc. It's also used to set up the enums that will switch between the 
different modes (Disables,Auto,etc). This program is mainly there so other parts of the program can refrence different components from this file */ 
 

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

  //Define components from the components classes and names them
  private Climber climber;
  private Drivebase drivebase;
  private Elevator elevator;
  private Arm arm;
  private Intake intake;

  private Pneumatics pneumatics;

  //Define Robot control modes
  private Teleop teleop;
  private Auton auton;

  //Extra stuff like vision + dashboard
  private Vision vision;

  private Dashboard dashboard;

  //Defines a list of robot components
  private List<Titan.Component<Robot>> components = List.of();

  @Override
  public void robotInit() {
    Titan.DEBUG = false;
    //Titan.DEBUG = Constants.ROBOT_TYPE == Constants.Robot.PRACTICE;

    Titan.DEBUG = true;

    //compressor.stop();


    //Actually creates the object instance of components defined earlier
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

    //Actually puts the objects of the components in the list
    components = List.of(teleop, auton, dashboard, vision, arm, climber, drivebase, elevator, intake, pneumatics);
  }

  //Based on the mode from the driver station it will switch all the robot components to that mode with the enums (Disable,Auto,etc)
  @Override
  public void robotPeriodic() {
    components.forEach((com)->com.tick(this)); //
  }
  
  @Override
  public void teleopInit() {
    mode = Mode.TELEOP;
    components.forEach((com)->com.init(this));
  }

  @Override
  public void teleopPeriodic() {
    //compressor.stop();

    components.forEach((com)->com.periodic(this));
  }

  //Since the autonomous period is the sandstorm, and it uses the same code as teleop, to not repeat code, we just call the teleop methods from autonomous.

  @Override
  public void autonomousInit() {
    mode = Mode.AUTO;
    components.forEach((com)->com.init(this));
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  @Override
  public void testInit(){
    mode = Mode.TEST;
    components.forEach((com)->com.init(this));
  }

  @Override
  public void testPeriodic(){
    teleopPeriodic();
  }

  @Override
  public void disabledInit(){
    mode = Mode.DISABLED;
    components.forEach((com)->com.disabled(this));
  }


  //Getter methods
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
    return components;
  }
}