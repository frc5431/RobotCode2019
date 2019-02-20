package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.components.Climber;
import frc.robot.components.Drivebase;
import frc.robot.components.Elevator;
import frc.robot.components.Arm;
import frc.robot.components.Intake;
import frc.robot.components.Teleop;
import frc.robot.components.Auton;

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

  private Compressor compressor;

  private Teleop teleop;
  private Auton auton;

  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture(0);
    CameraServer.getInstance().startAutomaticCapture(1);

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
  }

  @Override
  public void robotPeriodic() {
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

    elevator.periodic(this);
    intake.periodic(this);
    arm.periodic(this);
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
