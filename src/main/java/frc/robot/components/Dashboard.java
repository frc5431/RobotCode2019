package frc.robot.components;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

public class Dashboard {
    public Dashboard(){
        final UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture(0);
        camera1.setConnectVerbose(0);
        final UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
        camera2.setConnectVerbose(0);
    }

    public void periodic(final Robot robot){
        SmartDashboard.putNumber("HatchDistance", robot.getIntake().getHatchDistance());
        SmartDashboard.putNumber("WristPosition", robot.getArm().getWristPosition());
        SmartDashboard.putNumber("ElevatorPosition", robot.getElevator().getEncoderPosition());
        SmartDashboard.putBoolean("CarriageDown", robot.getElevator().isCarriageDown());
        SmartDashboard.putBoolean("CarriageUp", robot.getElevator().isCarriageUp());
        SmartDashboard.putBoolean("ElevatorDown", robot.getElevator().isElevatorDown());

        //SmartDashboard.putNumber("NavxAngle", robot.getDrivebase().getNavx().getAngle());

        SmartDashboard.putNumber("LeftDistance", robot.getDrivebase().getLeftDistance());
        SmartDashboard.putNumber("RightDistance", robot.getDrivebase().getRightDistance());

        SmartDashboard.putString("Mode", robot.getMode().toString());
    }
}