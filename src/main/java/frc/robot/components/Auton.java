package frc.robot.components;

import java.util.Map;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.lang.Integer;

import java.util.function.Function;

import frc.robot.Constants;
import frc.robot.util.ControlMode;
import frc.robot.Robot;
import frc.robot.util.Titan;

import frc.robot.auto.commands.ElevateToCommand;
import frc.robot.auto.commands.ArmMoveToCommand.CompletionCondition;
import frc.robot.auto.commands.ArmMoveToCommand;
import frc.robot.auto.commands.ArmBrakeCommand;
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

public class Auton extends Titan.Component<Robot>{
    private Titan.CommandQueue<Robot> sequenceCommands, drivebaseCommands, preloadedAutoCommands;
    private Titan.Joystick buttonBoard;

    private EnumMap<Sequence, Function<Robot, List<Titan.Command<Robot>>>> hatchSequences = new EnumMap<>(Sequence.class);
    private EnumMap<Sequence, Function<Robot, List<Titan.Command<Robot>>>> ballSequences = new EnumMap<>(Sequence.class);
    private HashMap<Integer, Sequence> sequences = new HashMap<>();

    private Sequence runningSequence = null;

    private boolean mimicLoaded = false;

    public Auton(){
        sequenceCommands = new Titan.CommandQueue<>();
        drivebaseCommands = new Titan.CommandQueue<>();
        preloadedAutoCommands = new Titan.CommandQueue<>();

        buttonBoard = new Titan.LogitechExtreme3D(Constants.BUTTONBOARD_JOYSTICK_ID);
        
        mimicLoaded = true;

        final Function<Robot, List<Titan.Command<Robot>>> stow = (robot)->goToPosition(robot, List.of(new Titan.ConsumerCommand<>((rob)->rob.getIntake().roll(0.0))), 0, Constants.ARM_STOW_ANGLE);

        hatchSequences.put(Sequence.STOW, stow);
        ballSequences.put(Sequence.STOW, stow);

        final Function<Robot, List<Titan.Command<Robot>>> climb = (robot)->{
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
            final List<Titan.Command<Robot>> deploymentSequence = List.of(new JayCommand(JayState.DEPLOYED), new FingerCommand(FingerState.RETRACTED));
            // if(Titan.approxEquals(robot.getElevator().getEncoderPosition(), hatchLoadingStationPos, 500) && Titan.approxEquals(robot.getArm().getArmAngle(), 95, 5)){
            //     return deploymentSequence;
            // }
            return goToPosition(robot, deploymentSequence, 0, 116);
            //return goToPosition(robot, deploymentSequence, 0.1085, 100);
        });

        //keep the hatch inside when moving the arm for the hatch rocket sequences
        final List<Titan.Command<Robot>> hatchRocketCustomCommands = List.of(new FingerCommand(FingerState.DEPLOYED));

        hatchSequences.put(Sequence.ROCKET_FORWARD_1, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0, 117));
        //flush: 8000
        hatchSequences.put(Sequence.ROCKET_FORWARD_2, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0.6177, 100));
        //flusH: 31000
        hatchSequences.put(Sequence.ROCKET_FORWARD_3, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0.9676, 110));
        //angled: 40000, 115
        //flush: 46000, 95
        hatchSequences.put(Sequence.CARGO_SHIP, hatchSequences.get(Sequence.ROCKET_FORWARD_1));

        // hatchSequences.put(Sequence.ROCKET_REVERSE_2, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0.5813, 240
        // ));

        // hatchSequences.put(Sequence.ROCKET_REVERSE_3, (robot)->goToPosition(robot, hatchRocketCustomCommands, 0.8510, 240));

        ballSequences.put(Sequence.ROCKET_FORWARD_1, (robot)->goToPosition(robot, 0.3311, 90));

        ballSequences.put(Sequence.ROCKET_FORWARD_2, (robot)->goToPosition(robot, 0.7410, 96));

        ballSequences.put(Sequence.ROCKET_FORWARD_3, (robot)->goToPosition(robot, 1.0010, 118));

        // ballSequences.put(Sequence.ROCKET_REVERSE_1, (robot)->goToPosition(robot, 0.5116, 270));

        ballSequences.put(Sequence.CARGO_SHIP, (robot)->goToPosition(robot, 0.5348, 95));

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
    }

    @Override
    public void init(final Robot robot){
        switch(robot.getMode()){
        case AUTO:
            robot.getDrivebase().resetAll();
            drivebaseCommands.clear();
            drivebaseCommands.add(new Titan.WaitCommand<>(100));
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
        
        drivebaseCommands.init(robot);
        sequenceCommands.init(robot);
    }

    @Override
    public void periodic(final Robot robot){
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

        final boolean leftBumperTriggered = robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.BUMPER_L);
        final boolean rightBumperTriggered = robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.BUMPER_R);
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
            drivebaseCommands.init(robot);
        }

        sequenceCommands.update(robot);
        drivebaseCommands.update(robot);
    }

    @Override
    public void disabled(final Robot robot){
        abort(robot);
    }

    public void abort(final Robot robot){
        sequenceCommands.done(robot);
        sequenceCommands.clear();

        drivebaseCommands.done(robot);
        drivebaseCommands.clear();

        robot.getDrivebase().setControlMode(ControlMode.MANUAL);
        robot.getArm().setControlMode(ControlMode.MANUAL);
        robot.getElevator().setControlMode(ControlMode.MANUAL);
        robot.getIntake().setControlMode(ControlMode.MANUAL);
    }

    public void runSequence(final Robot robot, final SequenceType type, final Sequence seq){
        sequenceCommands.done(robot);
        sequenceCommands.clear();
        runningSequence = seq;
        final Function<Robot, List<Titan.Command<Robot>>> selected = (type == SequenceType.HATCH ? hatchSequences : ballSequences).getOrDefault(seq, (rob)->List.of());
        if(type == SequenceType.CARGO){
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(new FingerCommand(FingerState.RETRACTED));
            queue.addCommand(new JayCommand(JayState.RETRACTED));
            queue.addQueue(new Titan.CommandQueue<>(selected.apply(robot)));
            sequenceCommands.add(queue);
        }else{
            sequenceCommands.addAll(selected.apply(robot));
        }
        sequenceCommands.init(robot);
    }

    private List<Titan.Command<Robot>> getAutoAim(final Sequence preSequence, final Sequence postSequence, final TargetType ttype){
        final List<Titan.Command<Robot>> out = new ArrayList<>();
        if(preSequence != null){
            out.add(new Titan.ConsumerCommand<>((rob)->{
                runSequence(rob, SequenceType.HATCH, preSequence);
            }));
            out.add(new Titan.ConditionalCommand<>((rob)->sequenceCommands.isEmpty()));
        }
        out.add(new DriveToTargetCommand(ttype));
        out.add(new Titan.ConsumerCommand<>((rob)->{
            runSequence(rob, SequenceType.HATCH, postSequence);
        }));
        return out;
    }

    public boolean isArmInStowPosition(final double armPos){
        return Titan.approxEquals(armPos, Constants.ARM_STOW_ANGLE, 10) || (armPos > 180 && armPos < Constants.ARM_PRESTOW_REVERSE_ANGLE);
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

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final double elevatorPos, final double armPos){
        return goToPosition(robot, List.of(), elevatorPos, armPos, List.of());
    }

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final List<Titan.Command<Robot>> preCommands, final double elevatorPos, final double armPos){
        return goToPosition(robot, preCommands, elevatorPos, armPos, List.of());
    }

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final double elevatorPos, final double armPos, final List<Titan.Command<Robot>> extraCommands){
        return goToPosition(robot, List.of(), elevatorPos, armPos, extraCommands);
    }

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final List<Titan.Command<Robot>> preCommands, final double target, final double targetArmPos, final List<Titan.Command<Robot>> postCommands){
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

        final List<Titan.Command<Robot>> out = new ArrayList<>();
        // can't move the elevator up or down if the arm is in a stowed position
        if((currentElevatorPos > 0 && isArmInStowPosition(targetArmPos) && isArmInStowPosition(currentArmPos)) || (targetElevatorPos > 0 && isArmInStowPosition(currentArmPos)) || (currentArmDirection != targetArmDirection && isArmInStowPosition(currentArmPos))){
            out.add(new ArmMoveToCommand(getStowAngle(currentArmDirection), CompletionCondition.TRAVELLED));
        }

        if(currentArmDirection != targetArmDirection){
            if(elevatorAllowsIntakeFlip(currentElevatorPos) && elevatorAllowsIntakeFlip(targetElevatorPos)){
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                queue.addCommand(new ElevateToCommand(targetElevatorPos));
                queue.addCommand(new ArmMoveToCommand(targetArmPos));
                out.add(queue);
            }else{
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                final Titan.CommandQueue<Robot> subArmQueue = new Titan.CommandQueue<>();
                if(robot.getArm().getArmAngle() < 80){
                    subArmQueue.add(new ArmMoveToCommand(90, CompletionCondition.TRAVELLED));
                }
                subArmQueue.add(new Titan.ConditionalCommand<>((rob)->elevatorAllowsIntakeFlip(rob.getElevator().getEncoderPosition())));
                if(isArmInStowPosition(targetArmPos)){
                    subArmQueue.add(new ArmMoveToCommand(getStowAngle(targetArmDirection)));
                }else{
                    subArmQueue.add(new ArmMoveToCommand(targetArmPos));
                }
                queue.addQueue(subArmQueue);
                if(elevatorAllowsIntakeFlip(targetElevatorPos)){
                    queue.addCommand(new ElevateToCommand(targetElevatorPos));
                }else/* if(!elevatorAllowsIntakeFlip(currentElevatorPos))*/{
                //}
                //if(!elevatorAllowsIntakeFlip(targetElevatorPos)){
                    final Titan.CommandQueue<Robot> subElevatorQueue = new Titan.CommandQueue<>();
                    subElevatorQueue.add(new ElevateToCommand(Constants.ELEVATOR_INTAKE_FLIP_LIMIT + (int)(0.1000 * Constants.ELEVATOR_ENCODER_CALIBRATION)));                    
                    if(targetArmDirection == ArmDirection.FORWARD){
                        subElevatorQueue.add(new Titan.ConditionalCommand<>((rob)->rob.getArm().getArmAngle() <= getStowAngle(targetArmDirection)));
                    }else{
                        subElevatorQueue.add(new Titan.ConditionalCommand<>((rob)->rob.getArm().getArmAngle() >= getStowAngle(targetArmDirection)));
                    }
                    subElevatorQueue.add(new ElevateToCommand(targetElevatorPos));
                    queue.addQueue(subElevatorQueue);
                }
                out.add(queue);
                out.add(new ArmMoveToCommand(targetArmPos));
            }
        }else /* if target is in the same direction as current*/{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            if(isInFloorIntake(targetElevatorPos, targetArmPos) && targetElevatorPos < Constants.ELEVATOR_BOTTOM_LIMIT){
                queue.addCommand(new ElevateToCommand(Constants.ELEVATOR_FLOOR_INTAKE_HEIGHT));
            }else{
                final Titan.CommandQueue<Robot> subQueue = new Titan.CommandQueue<>();
                if(isElevatorInStage2(targetElevatorPos) && isArmInStowPosition(currentArmPos)){
                    subQueue.add(new Titan.ConditionalCommand<>((rob)->!isArmInStowPosition(rob.getArm().getArmAngle())));
                }
                subQueue.add(new ElevateToCommand(targetElevatorPos));
                queue.addQueue(subQueue);
            }
            if(currentElevatorPos > 0 && isArmInStowPosition(targetArmPos)){
                queue.addCommand(new ArmMoveToCommand(getStowAngle(currentArmDirection), CompletionCondition.TRAVELLED));    
            }else if(isElevatorInStage2(targetElevatorPos) && Titan.approxEquals(90, targetArmPos, Constants.ARM_ANGLE_TOLERANCE) && currentArmPos > 120){
                queue.addCommand(new ArmMoveToCommand(getStowAngle(currentArmDirection), CompletionCondition.TRAVELLED));    
            }else{
                queue.addCommand(new ArmMoveToCommand(targetArmPos));
            }
            out.add(queue);

            if(isInFloorIntake(targetElevatorPos, targetArmPos)){
                out.add(new ElevateToCommand(targetElevatorPos));
                out.add(new ArmMoveToCommand(targetArmPos, CompletionCondition.TRAVELLED_NO_BRAKE));
            }else if(!isArmInStowPosition(targetArmPos) && targetElevatorPos > 0){
                /* when the elevator goes really high up, the arm sags because of the movement.
                these next two commands check when the elevator is done moving and corrects for any sag.
                */
                out.add(new Titan.ConditionalCommand<Robot>((rob)->Titan.approxEquals(0, rob.getElevator().getEncoderVelocity(), 2)));
                out.add(new ArmMoveToCommand(targetArmPos));
            }
        }

        // if we are in stow and done everything else, actually go to stow, or correct for any sag.
        if(isArmInStowPosition(targetArmPos) || (isElevatorInStage2(targetElevatorPos) && Titan.approxEquals(90, targetArmPos, Constants.ARM_ANGLE_TOLERANCE))){
            out.add(new ArmMoveToCommand(targetArmPos));
        }

        out.addAll(postCommands);

        if(preCommands.isEmpty()){
            return out;
        }else{
            final Titan.ParallelCommandGroup<Robot> outQueue = new Titan.ParallelCommandGroup<>();
            outQueue.addQueue(preCommands);
            outQueue.addQueue(out);
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
        //return buttonBoard.getRawAxis(Titan.LogitechExtreme3D.Axis.SLIDER) > 0 ? SequenceType.HATCH : SequenceType.CARGO;
        return buttonBoard.getRawButton(9) ? SequenceType.HATCH : SequenceType.CARGO;
    }

    public ArmDirection getTargetArmDirection(final Robot robot){
        if(runningSequence == null){
            return getArmDirection(robot.getArm().getArmAngle());
        }

        return runningSequence.getDirection();
    }

    public Titan.CommandQueue<Robot> getDrivebaseCommands(){
        return drivebaseCommands;
    }

    public Titan.CommandQueue<Robot> getSequenceCommands(){
        return sequenceCommands;
    }

    public Titan.CommandQueue<Robot> getPreloadedAutoCommands(){
        return preloadedAutoCommands;
    }

    public void preloadRoutine(final Routine r, final long delay){
        if(r == null || !mimicLoaded){
            return;
        }
        new Thread(()->{
            mimicLoaded = false;
            Titan.l("Preloading auto routine: " + r.toString());
            preloadedAutoCommands.clear();
            if(delay > 0){
                preloadedAutoCommands.add(new Titan.WaitCommand<>(delay));
            }
            final Path startHatchPath = r.getStartHatchPath();
            if(startHatchPath != null){
                final Sequence firstSequence = r.getFirstSequence();
                preloadedAutoCommands.add(new Titan.ConsumerCommand<>((rob)->{
                    rob.getVision().setTargetType(r.isSwapped() ? TargetType.FRONT_RIGHT : TargetType.FRONT_LEFT);
                }));
                preloadedAutoCommands.addAll(startHatchPath.generate(firstSequence, r.isSwapped()));
                if(firstSequence != null){
                    preloadedAutoCommands.add(new Titan.ConditionalCommand<>((rob)->!isRunningSequence()));
                    preloadedAutoCommands.addAll(getAutoAim(null, Sequence.OUTTAKE, r.isSwapped() ? TargetType.FRONT_RIGHT : TargetType.FRONT_LEFT));
                    preloadedAutoCommands.add(new Titan.WaitCommand<>(500));
                }
            }

            final Path loadingStationPath = r.getLoadingStationPath();
            if(loadingStationPath != null){
                preloadedAutoCommands.addAll(loadingStationPath.generate(Sequence.LOADING_STATION, r.isSwapped()));
                preloadedAutoCommands.add(new Titan.ConditionalCommand<>((rob)->!isRunningSequence()));
                preloadedAutoCommands.addAll(getAutoAim(null, Sequence.INTAKE, TargetType.FRONT_RIGHT));
            }

            final Path secondHatchPath = r.getSecondHatchPath();
            if(secondHatchPath != null){
                final Sequence secondSequence = r.getSecondSequence();
                preloadedAutoCommands.addAll(secondHatchPath.generate(secondSequence, r.isSwapped()));
                if(secondSequence != null){
                    preloadedAutoCommands.add(new Titan.ConditionalCommand<>((rob)->!isRunningSequence()));
                    preloadedAutoCommands.addAll(getAutoAim(null, Sequence.OUTTAKE, r.isSwapped() ? TargetType.FRONT_LEFT : TargetType.FRONT_RIGHT));
                }
            }
            Titan.l("Finished preloading");
            mimicLoaded = true;
        }).start();
    }

    public boolean hasPreloadedRoutine(){
        return !preloadedAutoCommands.isEmpty();
    }

    public Titan.Joystick getButtonBoard(){
        return buttonBoard;
    }

    public void setLEDStatus(final boolean on){
        buttonBoard.setOutput(3, on);
    }
}