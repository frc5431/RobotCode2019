package frc.robot.components;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Robot;
import frc.robot.util.Titan;
import frc.robot.util.Component;
import frc.robot.util.Testable;

public class Dashboard extends Component{
    public static enum MimicFile{
        FAR_ROCKET_2_FORWARD("farrocket_v1"),
        FAR_ROCKET_2_REVERSE("farrocket_v1", true),
        TEST("TEST");

        private final String file;
        private final boolean swapped;

        MimicFile(final String file){
            this(file, false);
        }

        MimicFile(final String file, final boolean swapped){
            this.file = file;
            this.swapped = swapped;
        }

        public String getFile(){
            return file;
        }

        public boolean isSwapped(){
            return swapped;
        }
    };

    private final SendableChooser<MimicFile> mimicChooser = new SendableChooser<>();

    private final Titan.Toggle selfTest = new Titan.Toggle();

    //private String frontCameraData = null;

    public Dashboard(){
        final CameraServer cameraServer = CameraServer.getInstance();

        final UsbCamera intakeCamera = cameraServer.startAutomaticCapture("Intake Camera", 0);
        intakeCamera.setConnectVerbose(0);
        intakeCamera.setFPS(15);
        intakeCamera.setResolution(96, 54);

        for(final MimicFile file : MimicFile.values()){
            mimicChooser.addOption(file.toString(), file);
        }
        mimicChooser.setDefaultOption(MimicFile.FAR_ROCKET_2_FORWARD.toString(), MimicFile.FAR_ROCKET_2_FORWARD);
        SmartDashboard.putData("MimicChooser", mimicChooser);
    }

    
    @Override
    public void init(final Robot robot){   
    }

    @Override
    public void periodic(final Robot robot){
        SmartDashboard.putData("MimicChooser", mimicChooser);

        robot.getAuton().getButtonBoard().setOutput(3, robot.getAuton().isRunningSequence() || (robot.getAuton().isRunningMimic() && System.currentTimeMillis() % 1000 > 500) || (robot.getAuton().isRecording() && System.currentTimeMillis() % 250 >= 125));

        SmartDashboard.putNumber("ArmAngle", robot.getArm().getArmAngle());
        SmartDashboard.putNumber("ElevatorPosition", robot.getElevator().getEncoderPosition());
        SmartDashboard.putNumber("ElevatorVelocity", robot.getElevator().getEncoderVelocity());
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
                builder.append(t.getClass().getName()).append(": ").append(t.getTestResult()).append(System.lineSeparator());
            }

            SmartDashboard.putString("SelfTest", builder.toString());
            
            // PERFORM SELF TEST
            selfTest.setState(false);
        }
    }

    @Override
    public void disabled(final Robot robot){
    }

    public MimicFile getChosenMimicFile(){
        return mimicChooser.getSelected();
    }

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}