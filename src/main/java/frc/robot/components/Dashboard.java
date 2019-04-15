package frc.robot.components;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
        for(final Routine r : Routine.values()){
            routineChooser.addOption(r.toString(), r);
        }
        routineChooser.setDefaultOption(Routine.HAB2_TO_FROCKET_2_RIGHT.toString(), Routine.HAB2_TO_FROCKET_2_RIGHT);
        SmartDashboard.putData("RoutineChooser", routineChooser);

        LiveWindow.disableAllTelemetry();
    }

    
    @Override
    public void init(final Robot robot){
        SmartDashboard.putData("RoutineChooser", routineChooser);
    }

    @Override
    public void periodic(final Robot robot){
        final Auton auton = robot.getAuton();
        auton.setLEDStatus(auton.isRunningSequence() || (auton.isRunningDrivebase() && System.currentTimeMillis() % 1000 > 500));
    }

    @Override
    public void tick(final Robot robot){
        SmartDashboard.putData("RoutineChooser", routineChooser);

        final Arm arm = robot.getArm();
        final Elevator elevator = robot.getElevator();
        final Drivebase drivebase = robot.getDrivebase();
        final Auton auton = robot.getAuton();

        SmartDashboard.putNumber("ArmAngle", arm.getAbsoluteAngle());
        SmartDashboard.putNumber("ArmEncoderPosition", arm.getEncoderPosition());
        SmartDashboard.putNumber("ArmEncoderVelocity", arm.getEncoderVelocity());
        SmartDashboard.putNumber("ArmPower", arm.getOutputPower());
        SmartDashboard.putNumber("ElevatorPosition", elevator.getEncoderPosition());
        SmartDashboard.putNumber("ElevatorVelocity", elevator.getEncoderVelocity());
        SmartDashboard.putBoolean("CarriageDown", elevator.isCarriageDown());
        SmartDashboard.putBoolean("CarriageUp", elevator.isCarriageUp());
        SmartDashboard.putBoolean("ElevatorDown", elevator.isElevatorDown());

        SmartDashboard.putNumber("NavxAngle", drivebase.getAngle());

        SmartDashboard.putNumber("LeftDistance", drivebase.getLeftDistance());
        SmartDashboard.putNumber("RightDistance", drivebase.getRightDistance());

        final TargetInfo info = robot.getVision().getTargetInfo();
        SmartDashboard.putNumber("TargetXAngle", info.getXAngle());
        SmartDashboard.putNumber("TargetYAngle", info.getYAngle());
        SmartDashboard.putNumber("TargetArea", info.getArea());
        SmartDashboard.putBoolean("TargetExists", info.exists());

        final Mode mode = robot.getMode();
        SmartDashboard.putString("Mode", mode.toString());

        final Routine selectedRoutine = routineChooser.getSelected();
        if(currentRoutine != selectedRoutine || (mode == Mode.DISABLED && !auton.hasPreloadedRoutine())){
            currentRoutine = selectedRoutine;
            auton.preloadRoutine(currentRoutine);
        }
    }

    @Override
    public void disabled(final Robot robot){
    }

    public Routine getChosenRoutine(){
        return routineChooser.getSelected();
    }
}