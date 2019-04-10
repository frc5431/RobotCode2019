package frc.robot.components;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Robot;
import frc.robot.Robot.Mode;
import frc.robot.util.Titan;
import frc.robot.auto.Routine;
import frc.robot.auto.vision.TargetInfo;

public class Dashboard extends Titan.Component<Robot>{
    private final SendableChooser<Routine> routineChooser = new SendableChooser<>();
    private Routine currentRoutine = null;

    public Dashboard(){
        final CameraServer cameraServer = CameraServer.getInstance();

        final UsbCamera intakeCamera = cameraServer.startAutomaticCapture("Intake Camera", 0);
        intakeCamera.setConnectVerbose(0);
        intakeCamera.setFPS(15);
        intakeCamera.setResolution(96, 54);

        for(final Routine r : Routine.values()){
            routineChooser.addOption(r.toString(), r);
        }
        routineChooser.setDefaultOption(Routine.ROCKET_TIER_2_RIGHT.toString(), Routine.ROCKET_TIER_2_RIGHT);
        SmartDashboard.putData("RoutineChooser", routineChooser);
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
        SmartDashboard.putData("RoutineChooser", routineChooser);

        SmartDashboard.putNumber("ArmAngle", robot.getArm().getArmAngle());
        SmartDashboard.putNumber("ArmEncoderPosition", robot.getArm().getEncoderPosition());
        SmartDashboard.putNumber("ArmEncoderVelocity", robot.getArm().getEncoderVelocity());
        SmartDashboard.putNumber("ElevatorPosition", robot.getElevator().getEncoderPosition());
        SmartDashboard.putNumber("ElevatorVelocity", robot.getElevator().getEncoderVelocity());
        SmartDashboard.putBoolean("CarriageDown", robot.getElevator().isCarriageDown());
        SmartDashboard.putBoolean("CarriageUp", robot.getElevator().isCarriageUp());
        SmartDashboard.putBoolean("ElevatorDown", robot.getElevator().isElevatorDown());

        SmartDashboard.putNumber("NavxAngle", robot.getDrivebase().getAngle());

        SmartDashboard.putNumber("LeftDistance", robot.getDrivebase().getLeftDistance());
        SmartDashboard.putNumber("RightDistance", robot.getDrivebase().getRightDistance());

        final TargetInfo info = robot.getVision().getTargetInfo();
        SmartDashboard.putNumber("TargetXAngle", info.getXAngle());
        SmartDashboard.putNumber("TargetYAngle", info.getYAngle());
        SmartDashboard.putNumber("TargetArea", info.getArea());
        SmartDashboard.putBoolean("TargetExists", info.exists());

        SmartDashboard.putString("Mode", robot.getMode().toString());

        if(currentRoutine != routineChooser.getSelected() || (robot.getMode() == Mode.DISABLED && !robot.getAuton().hasPreloadedRoutine())){
            currentRoutine = routineChooser.getSelected();
            robot.getAuton().preloadRoutine(currentRoutine);
        }
    }

    @Override
    public void disabled(final Robot robot){
    }

    public Routine getChosenRoutine(){
        return routineChooser.getSelected();
    }
}