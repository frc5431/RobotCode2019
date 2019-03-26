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
import frc.robot.components.Auton.Sequence;

public class Dashboard extends Component{
    public static enum MimicFile{
        CARGOSHIP_1_HATCH_RIGHT(Sequence.CARGO_SHIP, "hab_to_cargo", "cargo_to_ls", null),
        CARGOSHIP_1_HATCH_LEFT(Sequence.CARGO_SHIP, "hab_to_cargo", "cargo_to_ls", null, true),
        CARGOSHIP_2_HATCH_RIGHT(Sequence.CARGO_SHIP,"hab_to_cargo", "cargo_to_ls", "ls_to_cargo"),
        CARGOSHIP_2_HATCH_LEFT(Sequence.CARGO_SHIP,"hab_to_cargo", "cargo_to_ls", "ls_to_cargo", true),
        ROCKET_TIER_1_RIGHT(Sequence.ROCKET_FORWARD_1, "hab_to_frocket", "frocket_to_ls", "ls_to_crocket"),
        ROCKET_TIER_1_LEFT(Sequence.ROCKET_FORWARD_1, "hab_to_frocket", "frocket_to_ls", "ls_to_crocket", true),
        ROCKET_TIER_2_RIGHT(Sequence.ROCKET_FORWARD_2, "hab_to_frocket", "frocket_to_ls", "ls_to_crocket"),
        ROCKET_TIER_2_LEFT(Sequence.ROCKET_FORWARD_2, "hab_to_frocket", "frocket_to_ls", "ls_to_crocket", true),
        ROCKET_TIER_3_RIGHT(Sequence.ROCKET_FORWARD_3, "hab_to_frocket", "frocket_to_ls", "ls_to_crocket"),
        ROCKET_TIER_3_LEFT(Sequence.ROCKET_FORWARD_3, "hab_to_frocket", "frocket_to_ls", "ls_to_crocket", true),
        CARGOSHIP_1_ROCKET_FAR_1_RIGHT(Sequence.ROCKET_FORWARD_1, "hab_to_cargo", "cargo_to_ls", "ls_to_frocket"),
        CARGOSHIP_1_ROCKET_FAR_1_LEFT(Sequence.ROCKET_FORWARD_1, "hab_to_cargo", "cargo_to_ls", "ls_to_frocket", true),
        CARGOSHIP_1_ROCKET_CLOSE_1_RIGHT(Sequence.ROCKET_FORWARD_1, "hab_to_cargo", "cargo_to_ls", "ls_to_crocket"),
        CARGOSHIP_1_ROCKET_CLOSE_1_LEFT(Sequence.ROCKET_FORWARD_1, "hab_to_cargo", "cargo_to_ls", "ls_to_crocket", true),
        TEST(null, "TEST", null, null),
        DO_NOTHING(null, null, null, null);

        private final String startHatch, loadingStation, secondHatch;
        private final Sequence sequence;
        private final boolean swapped;

        MimicFile(final Sequence sequence, final String startHatch, final String loadingStation, final String secondHatch){
            this(sequence, startHatch, loadingStation, secondHatch, false);
        }

        MimicFile(final Sequence sequence, final String startHatch, final String loadingStation, final String secondHatch, final boolean swapped){
            this.sequence = sequence;
            this.startHatch = startHatch;
            this.loadingStation = loadingStation;
            this.secondHatch = secondHatch;
            this.swapped = swapped;
        }

        public Sequence getSequence(){
            return sequence;
        }

        public String getStartHatchFile(){
            return startHatch;
        }

        public String getLoadingStationFile(){
            return loadingStation;
        }

        public String getSecondHatchFile(){
            return secondHatch;
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
        mimicChooser.setDefaultOption(MimicFile.CARGOSHIP_2_HATCH_RIGHT.toString(), MimicFile.CARGOSHIP_2_HATCH_RIGHT);
        SmartDashboard.putData("MimicChooser", mimicChooser);
    }

    
    @Override
    public void init(final Robot robot){   
    }

    @Override
    public void periodic(final Robot robot){
        robot.getAuton().getButtonBoard().setOutput(3, robot.getAuton().isRunningSequence() || (robot.getAuton().isRunningMimic() && System.currentTimeMillis() % 1000 > 500) || (robot.getAuton().isRecording() && System.currentTimeMillis() % 250 >= 125));
    }

    @Override
    public void tick(final Robot robot){
        SmartDashboard.putData("MimicChooser", mimicChooser);

        SmartDashboard.putNumber("ArmAngle", robot.getArm().getArmAngle());
        SmartDashboard.putNumber("ElevatorPosition", robot.getElevator().getEncoderPosition());
        SmartDashboard.putNumber("ElevatorVelocity", robot.getElevator().getEncoderVelocity());
        SmartDashboard.putBoolean("CarriageDown", robot.getElevator().isCarriageDown());
        SmartDashboard.putBoolean("CarriageUp", robot.getElevator().isCarriageUp());
        SmartDashboard.putBoolean("ElevatorDown", robot.getElevator().isElevatorDown());

        //SmartDashboard.putNumber("NavxAngle", robot.getDrivebase().getNavx().getAngle());

        SmartDashboard.putNumber("LeftDistance", robot.getDrivebase().getLeftDistance());
        SmartDashboard.putNumber("RightDistance", robot.getDrivebase().getRightDistance());

        final Vision.TargetInfo info = robot.getVision().getTargetInfo();
        SmartDashboard.putNumber("TargetXAngle", info.getXAngle());
        SmartDashboard.putNumber("TargetYAngle", info.getYAngle());
        SmartDashboard.putNumber("TargetArea", info.getArea());
        SmartDashboard.putBoolean("TargetExists", info.exists());

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