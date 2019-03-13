package frc.robot.components;

import java.util.Map;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.io.File;
import java.lang.Integer;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.ControlMode;
import frc.robot.util.MimicPropertyValue;
import frc.robot.util.Testable;
import frc.robot.util.Component;
import frc.robot.Robot;
import frc.robot.util.Titan;

import frc.robot.commands.ElevateToCommand;
import frc.robot.commands.ArmMoveToCommand;
import frc.robot.commands.FingerCommand;
import frc.robot.commands.GrabBallCommand;
import frc.robot.commands.JayCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RollerCommand;
import frc.robot.components.Dashboard.MimicFile;
import frc.robot.components.Intake.FingerState;
import frc.robot.components.Intake.JayState;

public class Auton extends Component{
    public static enum Sequence {
        FLOOR,
        LOADING_STATION,
        CARGO_SHIP,
        ROCKET_FORWARD_1,
        ROCKET_FORWARD_2,
        ROCKET_FORWARD_3,
        ROCKET_REVERSE_1,
        ROCKET_REVERSE_2,
        ROCKET_REVERSE_3,
        STOW,
        CLIMB,
        INTAKE,
        OUTTAKE
    };

    public static enum SequenceType {
        HATCH, CARGO
    };

    private static enum ArmDirection{
        FORWARD, REVERSE
    };

    private Titan.CommandQueue<Robot> commands, mimicCommands;
    private Titan.Joystick buttonBoard;

    private EnumMap<Sequence, Supplier<List<Titan.Command<Robot>>>> hatchSequences = new EnumMap<>(Sequence.class);
    private EnumMap<Sequence, Supplier<List<Titan.Command<Robot>>>> ballSequences = new EnumMap<>(Sequence.class);
    private HashMap<Integer, Sequence> sequences = new HashMap<>();

    private Sequence runningSequence = null;

    private Map<String, List<Titan.Mimic.Step<MimicPropertyValue>>> mimicFiles = new HashMap<>();

    private Titan.Mimic.Observer<Robot, MimicPropertyValue> observer;

    public Auton(){
        commands = new Titan.CommandQueue<>();
        mimicCommands = new Titan.CommandQueue<>();

        buttonBoard = new Titan.LogitechExtreme3D(Constants.BUTTONBOARD_JOYSTICK_ID);

        observer = new Titan.Mimic.Observer<>();

        new Thread(()->{
            System.out.println("Beginning Mimic file sweep of " + Titan.Mimic.DEFAULT_MIMIC_DIRECTORY);
            final File folder = new File(Titan.Mimic.DEFAULT_MIMIC_DIRECTORY);
            if(!folder.exists()){
                System.err.println("Mimic directory not found!");
            }
            for(final File f : folder.listFiles()){
                final String name = f.getName().substring(0, f.getName().length() - ".mimic".length());
                System.out.println("Found Mimic file with name " + name);
                mimicFiles.put(name, Titan.Mimic.load(name, MimicPropertyValue.class));
            }
        }).start();
    }

    public List<Titan.Command<Robot>> loadMimicFile(final Robot robot, final String name){
        return loadMimicFile(robot, name, false);
    }

    public List<Titan.Command<Robot>> loadMimicFile(final Robot robot, final String name, final boolean swapped){
        final List<Titan.Command<Robot>> outCommands = new ArrayList<>();
        outCommands.add(new Titan.ConsumerCommand<>((rob)->{
            rob.getDrivebase().setHome();

            rob.getDrivebase().setControlMode(ControlMode.AUTO);
        }));

        int lastRunningSequence = -1;

        //Collect the mimic file
        //mimicChooser.getSelected()
        final List<Titan.Mimic.Step<MimicPropertyValue>> steps = mimicFiles.get(name);
        for(final Titan.Mimic.Step<MimicPropertyValue> step : steps){
            final List<Titan.Command<Robot>> out = new ArrayList<>();
            if(step.getBoolean(MimicPropertyValue.HOME)){
                out.add(new Titan.ConsumerCommand<>((rob)->{
                    rob.getDrivebase().reset();
                }));
            }

            final int stepSequence = step.getInteger(MimicPropertyValue.RUNNING_SEQUENCE);
            if(stepSequence >= 0 && stepSequence != lastRunningSequence){
                out.add(new Titan.ConsumerCommand<>((rob)->{
                    runSequence(rob, SequenceType.values()[step.getInteger(MimicPropertyValue.SEQUENCE_TYPE)], Sequence.values()[stepSequence]);
                }));
            }
            lastRunningSequence = stepSequence;

            final double leftPower, leftDistance;
            final double rightPower, rightDistance;

            if(swapped){
                leftPower = step.getDouble(MimicPropertyValue.RIGHT_POWER);
                rightPower = step.getDouble(MimicPropertyValue.LEFT_POWER);

                leftDistance = step.getDouble(MimicPropertyValue.RIGHT_DISTANCE);
                rightDistance = step.getDouble(MimicPropertyValue.LEFT_DISTANCE);
            }else{
                leftPower = step.getDouble(MimicPropertyValue.LEFT_POWER);
                rightPower = step.getDouble(MimicPropertyValue.RIGHT_POWER);

                leftDistance = step.getDouble(MimicPropertyValue.LEFT_DISTANCE);
                rightDistance = step.getDouble(MimicPropertyValue.RIGHT_DISTANCE);
            }

            out.add(new DriveCommand(leftPower, rightPower, leftDistance, rightDistance, step.getDouble(MimicPropertyValue.BATTERY)));

            if(out.size() == 1){
                outCommands.add(out.get(0));
            }else{
                final Titan.ParallelCommandGroup<Robot> group = new Titan.ParallelCommandGroup<>();
                for(final Titan.Command<Robot> com : out){
                    group.addCommand(robot, com);
                }
                outCommands.add(group);
            }
        }
        outCommands.add(new Titan.ConsumerCommand<>((rob)->{
            rob.getDrivebase().disableAllPID();
		    rob.getDrivebase().drive(0.0, 0.0);
        }));

        return outCommands;
    }

    private void runSequence(final Robot robot, final SequenceType type, final Sequence seq){
        runningSequence = seq;
        final Supplier<List<Titan.Command<Robot>>> selected = (type == SequenceType.HATCH ? hatchSequences : ballSequences).getOrDefault(seq, ()->List.of());
        if(type == SequenceType.CARGO){
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new FingerCommand(FingerState.RETRACTED));
            queue.addCommand(robot, new JayCommand(JayState.RETRACTED));
            queue.addQueue(robot, new Titan.CommandQueue<>(selected.get()));
            commands.add(queue);
        }else{
            commands.addAll(selected.get());
        }
        commands.init(robot);
    }

    public boolean isArmInStowPosition(final double armPos){
        return Titan.approxEquals(armPos, Constants.ARM_STOW_ANGLE, 5);
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
        out.addAll(preCommands);

        // can't move the elevator up or down if the arm is in a stowed position
        if((currentElevatorPos > 0 && isArmInStowPosition(targetArmPos) && isArmInStowPosition(currentArmPos)) || (targetElevatorPos > 0 && isArmInStowPosition(currentArmPos)) || (currentArmDirection != targetArmDirection && isArmInStowPosition(currentArmPos))){
            out.add(new ArmMoveToCommand(getStowAngle(currentArmDirection), Constants.AUTO_ARM_SPEED * 1.5));
        }

        if(currentArmDirection != targetArmDirection){
            if(elevatorAllowsIntakeFlip(currentElevatorPos) && elevatorAllowsIntakeFlip(targetElevatorPos)){
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                queue.addCommand(robot, new ElevateToCommand(targetElevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                queue.addCommand(robot, new ArmMoveToCommand(targetArmPos, Constants.AUTO_ARM_SPEED));
                out.add(queue);
            }else{
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                if(elevatorAllowsIntakeFlip(targetElevatorPos)){
                    queue.addCommand(robot, new ElevateToCommand(targetElevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                }else if(!elevatorAllowsIntakeFlip(currentElevatorPos)){
                    queue.addCommand(robot, new ElevateToCommand(Constants.ELEVATOR_INTAKE_FLIP_LIMIT + (int)(0.0500 * Constants.ELEVATOR_ENCODER_CALIBRATION), Constants.AUTO_ELEVATOR_SPEED));
                }
                final Titan.CommandQueue<Robot> subArmQueue = new Titan.CommandQueue<>();
                subArmQueue.add(new Titan.ConditionalCommand<>((rob)->elevatorAllowsIntakeFlip(rob.getElevator().getEncoderPosition())));
                if(isArmInStowPosition(targetArmPos)){
                    subArmQueue.add(new ArmMoveToCommand(getStowAngle(targetArmDirection), Constants.AUTO_ARM_SPEED));
                }else{
                    subArmQueue.add(new ArmMoveToCommand(targetArmPos, Constants.AUTO_ARM_SPEED));
                }
                queue.addQueue(robot, subArmQueue);
                if(!elevatorAllowsIntakeFlip(targetElevatorPos)){
                    final Titan.CommandQueue<Robot> subElevatorQueue = new Titan.CommandQueue<>();
                    subElevatorQueue.add(new Titan.ConditionalCommand<>((rob)->elevatorAllowsIntakeFlip(rob.getElevator().getEncoderPosition())));
                    if(targetArmDirection == ArmDirection.FORWARD){
                        subElevatorQueue.add(new Titan.ConditionalCommand<>((rob)->rob.getArm().getArmAngle() <= getStowAngle(targetArmDirection)));
                    }else{
                        subElevatorQueue.add(new Titan.ConditionalCommand<>((rob)->rob.getArm().getArmAngle() >= getStowAngle(targetArmDirection)));
                    }
                    subElevatorQueue.add(new ElevateToCommand(targetElevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                    queue.addQueue(robot, subElevatorQueue);
                }
                out.add(queue);
                
            }
        }else /* if target is in the same direction as current*/{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            if(isInFloorIntake(targetElevatorPos, targetArmPos) && targetElevatorPos < Constants.ELEVATOR_BOTTOM_LIMIT){
                queue.addCommand(robot, new ElevateToCommand(Constants.ELEVATOR_FLOOR_INTAKE_HEIGHT, Constants.AUTO_ELEVATOR_SPEED));
            }else{
                final Titan.CommandQueue<Robot> subQueue = new Titan.CommandQueue<>();
                if(isElevatorInStage2(targetElevatorPos) && isArmInStowPosition(currentArmPos)){
                    subQueue.add(new Titan.ConditionalCommand<>((rob)->!isArmInStowPosition(rob.getArm().getArmAngle())));
                }
                subQueue.add(new ElevateToCommand(targetElevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                queue.addQueue(robot, subQueue);
            }
            if(currentElevatorPos > 0 && isArmInStowPosition(targetArmPos)){
                queue.addCommand(robot, new ArmMoveToCommand(getStowAngle(currentArmDirection), Constants.AUTO_ARM_SPEED));    
            }else if(isElevatorInStage2(targetElevatorPos) && Titan.approxEquals(90, targetArmPos, Constants.ARM_ANGLE_TOLERANCE) && currentArmPos > 120){
                queue.addCommand(robot, new ArmMoveToCommand(getStowAngle(currentArmDirection), Constants.AUTO_ARM_SPEED));    
            }else{
                queue.addCommand(robot, new ArmMoveToCommand(targetArmPos, Constants.AUTO_ARM_SPEED));
            }
            out.add(queue);

            if(isInFloorIntake(targetElevatorPos, targetArmPos)){
                out.add(new ElevateToCommand(targetElevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                out.add(new ArmMoveToCommand(targetArmPos, Constants.AUTO_ARM_SPEED, false));
            }else if(!isArmInStowPosition(targetArmPos) && targetElevatorPos > 0){
                /* when the elevator goes really high up, the arm sags because of the movement.
                these next two commands check when the elevator is done moving and corrects for any sag.
                */
                out.add(new Titan.ConditionalCommand<Robot>((rob)->Titan.approxEquals(0, rob.getElevator().getEncoderVelocity(), 2)));
                out.add(new ArmMoveToCommand(targetArmPos, Constants.AUTO_ARM_SPEED));
            }
        }

        // if we are in stow and done everything else, actually go to stow, or correct for any sag.
        if(isArmInStowPosition(targetArmPos) || (isElevatorInStage2(targetElevatorPos) && Titan.approxEquals(90, targetArmPos, Constants.ARM_ANGLE_TOLERANCE))){
            out.add(new ArmMoveToCommand(targetArmPos, Constants.AUTO_ARM_SPEED));
        }

        out.addAll(postCommands);

        return out;
    }

    @Override
    public void init(final Robot robot){
        final Supplier<List<Titan.Command<Robot>>> stow =()->goToPosition(robot, List.of(new Titan.ConsumerCommand<>((rob)->rob.getIntake().roll(0.0))), 0, Constants.ARM_STOW_ANGLE);

        hatchSequences.put(Sequence.STOW, stow);
        ballSequences.put(Sequence.STOW, stow);

        final Supplier<List<Titan.Command<Robot>>> climb = ()->goToPosition(robot, 0, 260);

        hatchSequences.put(Sequence.CLIMB, climb);
        ballSequences.put(Sequence.CLIMB, climb);

        hatchSequences.put(Sequence.INTAKE, ()->{
            return List.of(new FingerCommand(FingerState.DEPLOYED));
        });
        hatchSequences.put(Sequence.OUTTAKE, ()->{
            return List.of(new JayCommand(JayState.RETRACTED), new FingerCommand(FingerState.RETRACTED));
        });

        ballSequences.put(Sequence.INTAKE, ()->{
            return List.of(new RollerCommand(Constants.INTAKE_ROLLER_SPEED, -1));
        });
        ballSequences.put(Sequence.OUTTAKE, ()->{
            return List.of(new RollerCommand(-Constants.INTAKE_ROLLER_SPEED, -1));
        });

        ballSequences.put(Sequence.FLOOR, ()->{
            if(isInFloorIntake(robot.getElevator().getEncoderPosition(), robot.getArm().getArmAngle())){
                return List.of(new GrabBallCommand());
            }
            return goToPosition(robot, List.of(new RollerCommand(Constants.INTAKE_ROLLER_SPEED, -1)), 0.1234, 80, List.of(new ArmMoveToCommand(80, Constants.AUTO_ARM_SPEED * 0.2, false), new GrabBallCommand()));
        });

        final double hatchLoadingStationPos = 0.1595;
        hatchSequences.put(Sequence.LOADING_STATION, ()->{
            final List<Titan.Command<Robot>> deploymentSequence = List.of(new JayCommand(JayState.DEPLOYED), new FingerCommand(FingerState.RETRACTED));
            if(Titan.approxEquals(robot.getElevator().getEncoderPosition(), hatchLoadingStationPos, 500) && Titan.approxEquals(robot.getArm().getArmAngle(), 90, 5)){
                return deploymentSequence;
            }
            return goToPosition(robot, deploymentSequence, hatchLoadingStationPos, 92);
        });

        //keep the hatch inside when moving the arm for the hatch rocket sequences
        final List<Titan.Command<Robot>> hatchRocketCustomCommands = List.of(new FingerCommand(FingerState.DEPLOYED));

        hatchSequences.put(Sequence.ROCKET_FORWARD_1, ()->goToPosition(robot, hatchRocketCustomCommands, 0, 115));
        //flush: 8000
        hatchSequences.put(Sequence.ROCKET_FORWARD_2, ()->goToPosition(robot, hatchRocketCustomCommands, 0.4025, 115));
        //flusH: 31000
        hatchSequences.put(Sequence.ROCKET_FORWARD_3, ()->goToPosition(robot, hatchRocketCustomCommands, 1.0, 90));
        //angled: 40000, 115
        //flush: 46000, 95
        hatchSequences.put(Sequence.CARGO_SHIP, hatchSequences.get(Sequence.ROCKET_FORWARD_1));

        hatchSequences.put(Sequence.ROCKET_REVERSE_2, ()->goToPosition(robot, hatchRocketCustomCommands, 0.5350, 262));

        hatchSequences.put(Sequence.ROCKET_REVERSE_3, ()->goToPosition(robot, hatchRocketCustomCommands, 0.8936, 250));

        ballSequences.put(Sequence.ROCKET_FORWARD_1, ()->goToPosition(robot, 0.3085, 92));

        ballSequences.put(Sequence.ROCKET_FORWARD_2, ()->goToPosition(robot, 0.7340, 92));

        ballSequences.put(Sequence.ROCKET_FORWARD_3, ()->goToPosition(robot, 0.9574, 112));

        ballSequences.put(Sequence.CARGO_SHIP, ()->goToPosition(robot, 0.4468, 95));

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

        if(robot.getMode() == Robot.Mode.AUTO){
            robot.getDrivebase().setHome();
            //mimicCommands.addAll(loadMimicFile(robot, "farrocket_v1"));
            final MimicFile file = robot.getDashboard().getChosenMimicFile();
            mimicCommands.addAll(loadMimicFile(robot, file.getFile(), file.isSwapped()));
        }else if(robot.getMode() == Robot.Mode.TEST){
            robot.getDrivebase().setHome();
            observer.prepare(SmartDashboard.getString("MimicRecordingName", "TEST"));
        }else if(robot.getMode() == Robot.Mode.TELEOP){
            abort(robot);
        }
        
        mimicCommands.init(robot);
        commands.init(robot);
    }

    @Override
    public void periodic(final Robot robot){
        final SequenceType sequenceType = getCurrentSequenceType();

        if(commands.isEmpty()){
            runningSequence = null;
            for(final Map.Entry<Integer, Sequence> e : sequences.entrySet()){
                if(buttonBoard.getRawButton(e.getKey())){
                    runSequence(robot, sequenceType, e.getValue());
                    break;
                }
            }
        }

        commands.update(robot);
        mimicCommands.update(robot);

        if(robot.getMode() == Robot.Mode.TEST){
            observer.addStep(robot, MimicPropertyValue.class);
        }
    }

    @Override
    public void disabled(final Robot robot){
        abort(robot);

        observer.save();
    }

    public void abort(final Robot robot){
        commands.done(robot);
        commands.clear();

        mimicCommands.done(robot);
        mimicCommands.clear();

        robot.getArm().setControlMode(ControlMode.MANUAL);
        robot.getElevator().setControlMode(ControlMode.MANUAL);
        robot.getIntake().setControlMode(ControlMode.MANUAL);
    }

    public boolean isRecording(){
        return observer.isRecording();
    }

    public boolean isRunningSequence(){
        return !commands.isEmpty();
    }

    public boolean isRunningMimic(){
        return !mimicCommands.isEmpty();
    }

    public Sequence getRunningSequence(){
        return runningSequence;
    }

    public SequenceType getCurrentSequenceType(){
        //return buttonBoard.getRawAxis(Titan.LogitechExtreme3D.Axis.SLIDER) > 0 ? SequenceType.HATCH : SequenceType.CARGO;
        return buttonBoard.getRawButton(9) ? SequenceType.HATCH : SequenceType.CARGO;
    }

    public Titan.CommandQueue<Robot> getMimicCommands(){
        return mimicCommands;
    }

    public Titan.CommandQueue<Robot> getCommands(){
        return commands;
    }

    public Titan.Joystick getButtonBoard(){
        return buttonBoard;
    }

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}