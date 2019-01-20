package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  private Teleop teleop;

  @Override
  public void robotInit() {
    teleop = new Teleop();
  }

  @Override
  public void robotPeriodic() {
  }

  
  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    teleop.periodic();
  }

  // Since the autonomous period is the sandstorm, and it uses the same code as teleop, to not repeat code, we just call the teleop methods from autonomous.

  @Override
  public void autonomousInit() {
    teleopInit();
  }

  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }
}
