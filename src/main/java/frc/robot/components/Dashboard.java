package frc.robot.components;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.util.Titan;
import frc.robot.util.Component;
import frc.robot.util.Testable;

public class Dashboard extends Component{
    private final Titan.Toggle selfTest = new Titan.Toggle();

    public Dashboard(){
        final UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
        camera1.setConnectVerbose(0);
        final UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
        camera2.setConnectVerbose(0);
    }

    
    @Override
    public void init(final Robot robot){   
    }

    @Override
    public void periodic(final Robot robot){
        robot.getAuton().getButtonBoard().setOutput(3, robot.getAuton().isRunningSequence() || (robot.getAuton().isRunningMimic() && System.currentTimeMillis() % 1000 > 500) || (robot.getAuton().isRecording() && System.currentTimeMillis() % 250 >= 125));

        SmartDashboard.putNumber("ArmAngle", robot.getArm().getArmAngle());
        SmartDashboard.putNumber("ElevatorPosition", robot.getElevator().getEncoderPosition());
        SmartDashboard.putBoolean("CarriageDown", robot.getElevator().isCarriageDown());
        SmartDashboard.putBoolean("CarriageUp", robot.getElevator().isCarriageUp());
        SmartDashboard.putBoolean("ElevatorDown", robot.getElevator().isElevatorDown());

        //SmartDashboard.putNumber("NavxAngle", robot.getDrivebase().getNavx().getAngle());

        SmartDashboard.putNumber("LeftDistance", robot.getDrivebase().getLeftDistance());
        SmartDashboard.putNumber("RightDistance", robot.getDrivebase().getRightDistance());

        SmartDashboard.putString("Mode", robot.getMode().toString());

        if(!selfTest.getState() && selfTest.isToggled(RobotController.getUserButton())){
            final List<Testable> testables = new ArrayList<>();
            testables.addAll(robot.getComponents());

            final StringBuilder builder = new StringBuilder();
            for(final Testable t : testables){
                builder.append(t.getTestResult()).append(System.lineSeparator());
            }

            SmartDashboard.putString("SelfTest", builder.toString());
            
            // PERFORM SELF TEST
            selfTest.setState(false);
        }
    }

    @Override
    public void disabled(final Robot robot){
        
    }

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}