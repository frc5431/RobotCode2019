package frc.robot.components;

import java.util.Map;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.lang.Integer;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.util.ControlMode;
import frc.team5431.titan.core.joysticks.LogitechExtreme3D;
import frc.team5431.titan.core.joysticks.Xbox;
import frc.team5431.titan.core.misc.Calc;
import frc.team5431.titan.core.misc.Logger;
import frc.robot.Robot;

import frc.robot.auto.commands.ElevateToCommand;
import frc.robot.auto.commands.ArmMoveToCommand.CompletionCondition;
import frc.robot.auto.commands.ArmMoveToCommand;
import frc.robot.auto.commands.ArmBrakeCommand;
import frc.robot.auto.commands.CommandQueue;
import frc.robot.auto.commands.FingerCommand;
import frc.robot.auto.commands.GrabBallCommand;
import frc.robot.auto.commands.JayCommand;
import frc.robot.auto.commands.DriveToTargetCommand;
import frc.robot.auto.commands.RollerCommand;
import frc.robot.auto.Routine;
import frc.robot.components.Intake.FingerState;
import frc.robot.components.Intake.JayState;
import frc.robot.auto.vision.TargetType;
import frc.robot.auto.Sequence;
import frc.robot.auto.SequenceType;
import frc.robot.auto.ArmDirection;
import frc.robot.auto.Path;

public class Auton extends SubsystemBase {
    private CommandQueue sequenceCommands, drivebaseCommands, preloadedAutoCommands;
    private Joystick buttonBoard;

    private EnumMap<Sequence, Function<Robot, List<CommandBase>>> hatchSequences = new EnumMap<>(Sequence.class);
    private EnumMap<Sequence, Function<Robot, List<CommandBase>>> ballSequences = new EnumMap<>(Sequence.class);
    private HashMap<Integer, Sequence> sequences = new HashMap<>();

    private Sequence runningSequence = null;

    private boolean mimicLoaded = false;

    public Auton(){
        sequenceCommands = new CommandQueue();
        drivebaseCommands = new CommandQueue();
        preloadedAutoCommands = new CommandQueue();

        buttonBoard = new LogitechExtreme3D(Constants.BUTTONBOARD_JOYSTICK_ID);
        
        mimicLoaded = true;

        final Function<Robot, List<CommandBase>> stow = (robot)->goToPosition(robot, List.of(new RunCommand(()->Robot.getRobot().getIntake().roll(0.0), Robot.getRobot().getIntake())), 0, Constants.ARM_STOW_ANGLE);

        hatchSequences.put(Sequence.STOW, stow);
        ballSequences.put(Sequence.STOW, stow);

        final Function<Robot, List<CommandBase>> climb = (robot)->{
            if(getArmDirection(robot.getArm().getArmAngle()) == ArmDirection.REVERSE){
                return goToPosition(robot, 0, 250, List.of(new JayCommand(JayState.DEPLOYED)));
            }else{
                return goToPosition(robot, 0.2444, 245, List.of(new JayCommand(JayState.RETRACTED)));
            }
        };

        hatchSequences.put(Sequence.CLIMB, climb);
        ballSequences.put(Sequence.CLIMB, climb);

        hatchSequences.put(Sequence.INTAKE, (robot)->{
            //if(robot.getIntake().getFingerState() == FingerState.RETRACTED){
            return List.of(new FingerCommand(FingerState.DEPLOYED));
            // }else{
            //     return List.of(new FingerCommand(FingerState.RETRACTED));   
            // }
        });
        hatchSequences.put(Sequence.OUTTAKE, (robot)->{
            return List.of(new JayCommand(JayState.RETRACTED), new FingerCommand(FingerState.RETRACTED));
        });

        // ballSequences.put(Sequence.INTAKE, ()->{
        //     return List.of(new RollerCommand(Constants.INTAKE_ROLLER_SPEED, -1));
        // });
        // ballSequences.put(Sequence.OUTTAKE, ()->{
        //     return List.of(new RollerCommand(-Constants.INTAKE_ROLLER_SPEED, -1));
        // });

        ballSequences.put(Sequence.FLOOR, (robot)->{
            if(isInFloorIntake(robot.getElevator().getEncoderPosition(), robot.getArm().getArmAngle())){
                return List.of(new GrabBallCommand());
            }
            return goToPosition(robot, List.of(new RollerCommand(Constants.INTAKE_ROLLER_SPEED, -1)), 0.0953, 80, List.of(new ArmBrakeCommand(false), new GrabBallCommand()));
        });

        hatchSequences.put(Sequence.LOADING_STATION, (robot)->{
            final List<CommandBase> deploymentSequence = List.of(new JayCommand(JayState.DEPLOYED), new FingerCommand(FingerState.RETRACTED));
            // if(Titan.approxEquals(robot.getElevator().getEncoderPosition(), hatchLoadingStationPos, 500) && Titan.approxEquals(robot.getArm().getArmAngle(), 95, 5)){
            //     return deploymentSequence;
            // }
            return goToPosition(robot, deploymentSequence, 0.1085, 105);
            //return goToPosition(robot, deploymentSequence, 0.0883, 106);

        });

        //keep the hatch inside when moving the arm for the hatch rocket sequences
        final List<CommandBase> hatchRocketCustomCommands = List.of(new FingerCommand(FingerState.DEPLOYED));

        hatchSequences.put(Sequence.ROCKET_FORWARD_1, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0, 120));
        //flush: 8000
        hatchSequences.put(Sequence.ROCKET_FORWARD_2, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0.6177, 100));
        //flusH: 31000
        hatchSequences.put(Sequence.ROCKET_FORWARD_3, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0.977, 110));
        //angled: 40000, 115
        //flush: 46000, 95
        hatchSequences.put(Sequence.CARGO_SHIP, hatchSequences.get(Sequence.ROCKET_FORWARD_1));

        // hatchSequences.put(Sequence.ROCKET_REVERSE_2, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0.5813, 240
        // ));

        // hatchSequences.put(Sequence.ROCKET_REVERSE_3, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0.8510, 240));

        ballSequences.put(Sequence.ROCKET_FORWARD_1, (robot)->goToPosition(robot, 0.3311, 90));

        ballSequences.put(Sequence.ROCKET_FORWARD_2, (robot)->goToPosition(robot, 0.7557, 91.07));

        ballSequences.put(Sequence.ROCKET_FORWARD_3, (robot)->goToPosition(robot, 0.931, 125.62));

        // ballSequences.put(Sequence.ROCKET_REVERSE_1, (robot)->goToPosition(robot, 0.5116, 270));

        ballSequences.put(Sequence.CARGO_SHIP, (robot)->goToPosition(robot, 0.54, 95));

         // BUTTON MAPPINGS
        // LOGITECH
        // sequences.put(Titan.LogitechExtreme3D.Button.TRIGGER.ordinal() + 1, Sequence.STOW);

        // sequences.put(Titan.LogitechExtreme3D.Button.FIVE.ordinal() + 1, Sequence.FLOOR);
        // sequences.put(Titan.LogitechExtreme3D.Button.THREE.ordinal() + 1, Sequence.LOADING_STATION);
        // sequences.put(Titan.LogitechExtreme3D.Button.SIX.ordinal() + 1, Sequence.CARGO_SHIP);

        // sequences.put(Titan.LogitechExtreme3D.Button.TWELVE.ordinal() + 1, Sequence.ROCKET_FORWARD_1);
        // sequences.put(Titan.LogitechExtreme3D.Button.TEN.ordinal() + 1, Sequence.ROCKET_FORWARD_2);
        // sequences.put(Titan.LogitechExtreme3D.Button.EIGHT.ordinal() + 1, Sequence.ROCKET_FORWARD_3);

        // sequences.put(Titan.LogitechExtreme3D.Button.ELEVEN.ordinal() + 1, Sequence.ROCKET_REVERSE_1);
        // sequences.put(Titan.LogitechExtreme3D.Button.NINE.ordinal() + 1, Sequence.ROCKET_REVERSE_2);
        // sequences.put(Titan.LogitechExtreme3D.Button.SEVEN.ordinal() + 1, Sequence.ROCKET_REVERSE_3);

        // sequences.put(Titan.LogitechExtreme3D.Button.TWO.ordinal() + 1, Sequence.CLIMB);

        //BUTTON BOARD

        //nine is outtake
        //fourteen is intake
        sequences.put(12, Sequence.STOW);

        sequences.put(11, Sequence.INTAKE);
        sequences.put(4, Sequence.OUTTAKE);

        sequences.put(14, Sequence.FLOOR);
        sequences.put(8, Sequence.LOADING_STATION);
        sequences.put(16, Sequence.CARGO_SHIP);

        sequences.put(1, Sequence.ROCKET_FORWARD_1);
        sequences.put(2, Sequence.ROCKET_FORWARD_2);
        sequences.put(3, Sequence.ROCKET_FORWARD_3);

        sequences.put(6, Sequence.ROCKET_REVERSE_1);
        sequences.put(5, Sequence.ROCKET_REVERSE_2);
        sequences.put(7, Sequence.ROCKET_REVERSE_3);

        sequences.put(13, Sequence.CLIMB);
        //switch is 16

        Robot robot = Robot.getRobot();
        
        switch(robot.getMode()){
            case AUTO:
                robot.getDrivebase().resetAll();
                drivebaseCommands.clear();
                drivebaseCommands.add(new WaitCommand(0.1));
                drivebaseCommands.addAll(preloadedAutoCommands);
                preloadedAutoCommands.clear();
                break;
            case TEST:
                robot.getDrivebase().resetAll();
                break;
            case TELEOP:
            case DISABLED:
            default:
                abort(robot);
                break;
        }
        
        drivebaseCommands.initialize();
        sequenceCommands.initialize();
    }

    @Override
    public void periodic(){
        Robot robot = Robot.getRobot();
        if(sequenceCommands.isEmpty()){
            runningSequence = null;
        }

        for(final Map.Entry<Integer, Sequence> e : sequences.entrySet()){
            final Sequence requestedSequence = e.getValue();
            if(buttonBoard.getRawButton(e.getKey()) && requestedSequence != runningSequence){
                runSequence(robot, getCurrentSequenceType(), requestedSequence);
                break;
            }
        }

        final boolean leftBumperTriggered = robot.getTeleop().getDriver().getRawButton(Xbox.Button.BUMPER_L);
        final boolean rightBumperTriggered = robot.getTeleop().getDriver().getRawButton(Xbox.Button.BUMPER_R);
        if(drivebaseCommands.isEmpty() && (leftBumperTriggered || rightBumperTriggered)){
            final Sequence preSequence = isArmInStowPosition(robot.getArm().getArmAngle()) && robot.getElevator().getEncoderPosition() <= 2000 ? Sequence.ROCKET_FORWARD_1 : null;
            final TargetType ttype;
            if(leftBumperTriggered){
                ttype = TargetType.FRONT_LEFT;
            }else{
                ttype = TargetType.FRONT_RIGHT;
            }
            drivebaseCommands.addAll(getAutoAim(preSequence, robot.getIntake().getFingerState() == FingerState.DEPLOYED ? Sequence.OUTTAKE : Sequence.INTAKE, ttype));
            //drivebaseCommands.add(new MoveToTargetCommand());
            drivebaseCommands.initialize();
        }

        sequenceCommands.execute();
        drivebaseCommands.execute();
    }

    public void abort(final Robot robot){
        sequenceCommands.end(true);
        sequenceCommands.clear();

        drivebaseCommands.end(true);
        drivebaseCommands.clear();

        robot.getDrivebase().setControlMode(ControlMode.MANUAL);
        robot.getArm().setControlMode(ControlMode.MANUAL);
        robot.getElevator().setControlMode(ControlMode.MANUAL);
        robot.getIntake().setControlMode(ControlMode.MANUAL);
    }

    public void runSequence(final Robot robot, final SequenceType type, final Sequence seq){
        sequenceCommands.end(false);
        sequenceCommands.clear();
        runningSequence = seq;
        final Function<Robot, List<CommandBase>> selected = (type == SequenceType.HATCH ? hatchSequences : ballSequences).getOrDefault(seq, (rob)->List.of());
        if(type == SequenceType.CARGO){
            final ParallelCommandGroup queue = new ParallelCommandGroup();
            queue.addCommands(new FingerCommand(FingerState.RETRACTED));
            queue.addCommands(new JayCommand(JayState.RETRACTED));
            queue.addCommands(selected.apply(robot).toArray(new CommandBase[0]));
            sequenceCommands.add(queue);
        }else{
            sequenceCommands.addAll(selected.apply(robot));
        }
        sequenceCommands.initialize();
    }

    private List<CommandBase> getAutoAim(final Sequence preSequence, final Sequence postSequence, final TargetType ttype){
        final List<CommandBase> out = new ArrayList<>();
        if(preSequence != null){
            out.add(new RunCommand(()->{
                runSequence(Robot.getRobot(), SequenceType.HATCH, preSequence);
            }));
            out.add(new WaitUntilCommand(()->sequenceCommands.isEmpty()));
        }
        out.add(new DriveToTargetCommand(ttype));
        out.add(new RunCommand(()->{
            runSequence(Robot.getRobot(), SequenceType.HATCH, postSequence);
        }));
        return out;
    }

    public boolean isArmInStowPosition(final double armPos){
        return Calc.approxEquals(armPos, Constants.ARM_STOW_ANGLE, 10) || (armPos > 180 && armPos < Constants.ARM_PRESTOW_REVERSE_ANGLE);
    }

    public boolean isElevatorInStage2(final int elevatorPos){
        return elevatorPos > Constants.ELEVATOR_FIRST_STAGE_LIMIT;
    }

    public boolean isInFloorIntake(final int elevatorPos, final double armPos){
        return (elevatorPos < Constants.ELEVATOR_FLOOR_INTAKE_HEIGHT && armPos < 100) || (!isElevatorInStage2(elevatorPos) && armPos <= 80);
    }

    public boolean elevatorAllowsIntakeFlip(final int elevatorPos){
        return elevatorPos > Constants.ELEVATOR_INTAKE_FLIP_LIMIT;
    }

    public ArmDirection getArmDirection(final double armPos){
        return armPos > 180 ? ArmDirection.REVERSE : ArmDirection.FORWARD;
    }

    public double getStowAngle(final ArmDirection dir){
        return dir == ArmDirection.FORWARD ? Constants.ARM_PRESTOW_FORWARD_ANGLE : Constants.ARM_PRESTOW_REVERSE_ANGLE;
    }

    public List<CommandBase> goToPosition(final Robot robot, final double elevatorPos, final double armPos){
        return goToPosition(robot, List.of(), elevatorPos, armPos, List.of());
    }

    public List<CommandBase> goToPosition(final Robot robot, final List<CommandBase> preCommands, final double elevatorPos, final double armPos){
        return goToPosition(robot, preCommands, elevatorPos, armPos, List.of());
    }

    public List<CommandBase> goToPosition(final Robot robot, final double elevatorPos, final double armPos, final List<CommandBase> extraCommands){
        return goToPosition(robot, List.of(), elevatorPos, armPos, extraCommands);
    }

    public List<CommandBase> goToPosition(final Robot robot, final List<CommandBase> preCommands, final double target, final double targetArmPos, final List<CommandBase> postCommands){
        /*
        When reading this code, remember two important constraints for the range of motion of the robot.
        If any of these constraints aren't followed, the robot will break.

        1. The elevator can NEVER move while the arm is perfectly upright/perpendicular to the floor (or in stow)
        2. The arm can not flip if the elevator is not high enough
        */
        
        final ArmDirection currentArmDirection = getArmDirection(robot.getArm().getArmAngle());
        final ArmDirection targetArmDirection = getArmDirection(targetArmPos);

        final int targetElevatorPos = (int)(target * Constants.ELEVATOR_ENCODER_CALIBRATION);

        final int currentElevatorPos = robot.getElevator().getEncoderPosition();
        final double currentArmPos = robot.getArm().getArmAngle();

        final List<CommandBase> out = new ArrayList<>();
        // can't move the elevator up or down if the arm is in a stowed position
        if((currentElevatorPos > 0 && isArmInStowPosition(targetArmPos) && isArmInStowPosition(currentArmPos)) || (targetElevatorPos > 0 && isArmInStowPosition(currentArmPos)) || (currentArmDirection != targetArmDirection && isArmInStowPosition(currentArmPos))){
            out.add(new ArmMoveToCommand(getStowAngle(currentArmDirection), CompletionCondition.TRAVELLED));
        }

        if(currentArmDirection != targetArmDirection){
            if(elevatorAllowsIntakeFlip(currentElevatorPos) && elevatorAllowsIntakeFlip(targetElevatorPos)){
                final ParallelCommandGroup queue = new ParallelCommandGroup();
                queue.addCommands(new ElevateToCommand(targetElevatorPos));
                queue.addCommands(new ArmMoveToCommand(targetArmPos));
                out.add(queue);
            }else{
                final ParallelCommandGroup queue = new ParallelCommandGroup();
                final CommandQueue subArmQueue = new CommandQueue();
                if(robot.getArm().getArmAngle() < 80){
                    subArmQueue.add(new ArmMoveToCommand(90, CompletionCondition.TRAVELLED));
                }
                subArmQueue.add(new WaitUntilCommand(()->elevatorAllowsIntakeFlip(Robot.getRobot().getElevator().getEncoderPosition())));
                if(isArmInStowPosition(targetArmPos)){
                    subArmQueue.add(new ArmMoveToCommand(getStowAngle(targetArmDirection)));
                }else{
                    subArmQueue.add(new ArmMoveToCommand(targetArmPos));
                }
                queue.addCommands(subArmQueue.getCommands().toArray(new Command[0]));
                if(elevatorAllowsIntakeFlip(targetElevatorPos)){
                    queue.addCommands(new ElevateToCommand(targetElevatorPos));
                }else/* if(!elevatorAllowsIntakeFlip(currentElevatorPos))*/{
                //}
                //if(!elevatorAllowsIntakeFlip(targetElevatorPos)){
                    final CommandQueue subElevatorQueue = new CommandQueue();
                    subElevatorQueue.add(new ElevateToCommand(Constants.ELEVATOR_INTAKE_FLIP_LIMIT + (int)(0.1000 * Constants.ELEVATOR_ENCODER_CALIBRATION)));                    
                    if(targetArmDirection == ArmDirection.FORWARD){
                        subElevatorQueue.add(new WaitUntilCommand(()->Robot.getRobot().getArm().getArmAngle() <= getStowAngle(targetArmDirection)));
                    }else{
                        subElevatorQueue.add(new WaitUntilCommand(()->Robot.getRobot().getArm().getArmAngle() >= getStowAngle(targetArmDirection)));
                    }
                    subElevatorQueue.add(new ElevateToCommand(targetElevatorPos));
                    queue.addCommands(subElevatorQueue.getCommands().toArray(new Command[0]));
                }
                out.add(queue);
                out.add(new ArmMoveToCommand(targetArmPos));
            }
        }else /* if target is in the same direction as current*/{
            final ParallelCommandGroup queue = new ParallelCommandGroup();
            if(isInFloorIntake(targetElevatorPos, targetArmPos) && targetElevatorPos < Constants.ELEVATOR_BOTTOM_LIMIT){
                queue.addCommands(new ElevateToCommand(Constants.ELEVATOR_FLOOR_INTAKE_HEIGHT));
            }else{
                final CommandQueue subQueue = new CommandQueue();
                if(isElevatorInStage2(targetElevatorPos) && isArmInStowPosition(currentArmPos)){
                    subQueue.add(new WaitUntilCommand(()->!isArmInStowPosition(Robot.getRobot().getArm().getArmAngle())));
                }
                subQueue.add(new ElevateToCommand(targetElevatorPos));
                queue.addCommands(subQueue.getCommands().toArray(new Command[0]));
            }
            if(currentElevatorPos > 0 && isArmInStowPosition(targetArmPos)){
                queue.addCommands(new ArmMoveToCommand(getStowAngle(currentArmDirection), CompletionCondition.TRAVELLED));    
            }else if(isElevatorInStage2(targetElevatorPos) && Calc.approxEquals(90, targetArmPos, Constants.ARM_ANGLE_TOLERANCE) && currentArmPos > 120){
                queue.addCommands(new ArmMoveToCommand(getStowAngle(currentArmDirection), CompletionCondition.TRAVELLED));    
            }else{
                queue.addCommands(new ArmMoveToCommand(targetArmPos));
            }
            out.add(queue);

            if(isInFloorIntake(targetElevatorPos, targetArmPos)){
                out.add(new ElevateToCommand(targetElevatorPos));
                out.add(new ArmMoveToCommand(targetArmPos, CompletionCondition.TRAVELLED_NO_BRAKE));
            }else if(!isArmInStowPosition(targetArmPos) && targetElevatorPos > 0){
                /* when the elevator goes really high up, the arm sags because of the movement.
                these next two commands check when the elevator is done moving and corrects for any sag.
                */
                out.add(new WaitUntilCommand(()->Calc.approxEquals(0, Robot.getRobot().getElevator().getEncoderVelocity(), 2)));
                out.add(new ArmMoveToCommand(targetArmPos));
            }
        }

        // if we are in stow and done everything else, actually go to stow, or correct for any sag.
        if(isArmInStowPosition(targetArmPos) || (isElevatorInStage2(targetElevatorPos) && Calc.approxEquals(90, targetArmPos, Constants.ARM_ANGLE_TOLERANCE))){
            out.add(new ArmMoveToCommand(targetArmPos));
        }

        out.addAll(postCommands);

        if(preCommands.isEmpty()){
            return out;
        }else{
            final ParallelCommandGroup outQueue = new ParallelCommandGroup();
            outQueue.addCommands(preCommands.toArray(new Command[0]));
            outQueue.addCommands(out.toArray(new Command[0]));
            return List.of(outQueue);
        }
    }

    public boolean isRunningSequence(){
        return !sequenceCommands.isEmpty();
    }

    public boolean isRunningDrivebase(){
        return !drivebaseCommands.isEmpty();
    }

    public boolean isSequencePressed(final SequenceType type, final Sequence seq){
        if(getCurrentSequenceType() != type){
            return false;
        }
        for(final Map.Entry<Integer, Sequence> e : sequences.entrySet()){
            if(e.getValue() == seq && buttonBoard.getRawButton(e.getKey())){
                return true;
            }
        }
        return false;
    }

    public Sequence getRunningSequence(){
        return runningSequence;
    }

    public SequenceType getCurrentSequenceType(){
        //return buttonBoard.getRawAxis(LogitechExtreme3D.Axis.SLIDER) > 0 ? SequenceType.HATCH : SequenceType.CARGO;
        return buttonBoard.getRawButton(9) ? SequenceType.HATCH : SequenceType.CARGO;
    }

    public ArmDirection getTargetArmDirection(final Robot robot){
        if(runningSequence == null){
            return getArmDirection(robot.getArm().getArmAngle());
        }

        return runningSequence.getDirection();
    }

    public CommandQueue getDrivebaseCommands(){
        return drivebaseCommands;
    }

    public CommandQueue getSequenceCommands(){
        return sequenceCommands;
    }

    public CommandQueue getPreloadedAutoCommands(){
        return preloadedAutoCommands;
    }

    public void preloadRoutine(final Routine r, final long delay){
        if(r == null || !mimicLoaded){
            return;
        }
        new Thread(()->{
            mimicLoaded = false;
            Logger.l("Preloading auto routine: " + r.toString());
            preloadedAutoCommands.clear();
            if(delay > 0){
                preloadedAutoCommands.add(new WaitCommand(delay));
            }
            final Path startHatchPath = r.getStartHatchPath();
            if(startHatchPath != null){
                final Sequence firstSequence = r.getFirstSequence();
                preloadedAutoCommands.add(new RunCommand(()->{
                    Robot.getRobot().getVision().setTargetType(r.isSwapped() ? TargetType.FRONT_RIGHT : TargetType.FRONT_LEFT);
                }));
                preloadedAutoCommands.addAll(startHatchPath.generate(firstSequence, r.isSwapped()));
                if(firstSequence != null){
                    preloadedAutoCommands.add(new WaitUntilCommand(()->!isRunningSequence()));
                    preloadedAutoCommands.addAll(getAutoAim(null, Sequence.OUTTAKE, r.isSwapped() ? TargetType.FRONT_RIGHT : TargetType.FRONT_LEFT));
                    preloadedAutoCommands.add(new WaitCommand(0.5));
                }
            }

            final Path loadingStationPath = r.getLoadingStationPath();
            if(loadingStationPath != null){
                preloadedAutoCommands.addAll(loadingStationPath.generate(Sequence.LOADING_STATION, r.isSwapped()));
                preloadedAutoCommands.add(new WaitUntilCommand(()->!isRunningSequence()));
                preloadedAutoCommands.addAll(getAutoAim(null, Sequence.INTAKE, TargetType.FRONT_RIGHT));
            }

            final Path secondHatchPath = r.getSecondHatchPath();
            if(secondHatchPath != null){
                final Sequence secondSequence = r.getSecondSequence();
                preloadedAutoCommands.addAll(secondHatchPath.generate(secondSequence, r.isSwapped()));
                if(secondSequence != null){
                    preloadedAutoCommands.add(new WaitUntilCommand(()->!isRunningSequence()));
                    preloadedAutoCommands.addAll(getAutoAim(null, Sequence.OUTTAKE, r.isSwapped() ? TargetType.FRONT_LEFT : TargetType.FRONT_RIGHT));
                }
            }
            Logger.l("Finished preloading");
            mimicLoaded = true;
        }).start();
    }

    public boolean hasPreloadedRoutine(){
        return !preloadedAutoCommands.isEmpty();
    }

    public Joystick getButtonBoard(){
        return buttonBoard;
    }

    public void setLEDStatus(final boolean on){
        buttonBoard.setOutput(3, on);
    }
}