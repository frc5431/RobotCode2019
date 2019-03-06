package frc.robot.components;

import java.util.Map;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.lang.Integer;

import java.util.function.Supplier;

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
import frc.robot.commands.JayCommand;
import frc.robot.commands.HatchOuttakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RollerCommand;

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

    private Titan.Mimic.Observer<Robot, MimicPropertyValue> observer;

    private long lastIntake = -1;

    public Auton(){
        commands = new Titan.CommandQueue<>();
        mimicCommands = new Titan.CommandQueue<>();

        buttonBoard = new Titan.LogitechExtreme3D(Constants.BUTTONBOARD_JOYSTICK_ID);

        observer = new Titan.Mimic.Observer<>();
    }

    public List<Titan.Command<Robot>> loadMimicFile(final Robot robot, final String file){
        final List<Titan.Command<Robot>> outCommands = new ArrayList<>();
        outCommands.add(new Titan.ConsumerCommand<>((rob)->{
            rob.getDrivebase().setHome();

            robot.getDrivebase().setControlMode(ControlMode.AUTO);
        }));

        int lastRunningSequence = -1;

        //Collect the mimic file
        final ArrayList<Titan.Mimic.Step<MimicPropertyValue>> steps = Titan.Mimic.load(file.toLowerCase(), MimicPropertyValue.class);
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

            out.add(new DriveCommand(step.getDouble(MimicPropertyValue.LEFT_POWER), step.getDouble(MimicPropertyValue.RIGHT_POWER), step.getDouble(MimicPropertyValue.LEFT_DISTANCE), step.getDouble(MimicPropertyValue.RIGHT_DISTANCE), step.getDouble(MimicPropertyValue.BATTERY)));

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
        commands.addAll(selected.get());
        commands.init(robot);
    }

    public boolean isArmInStowPosition(final double armPos){
        return Titan.approxEquals(armPos, Constants.ARM_STOW_ANGLE, Constants.ARM_ANGLE_TOLERANCE);
    }

    public boolean isElevatorInStage2(final int elevatorPos){
        return elevatorPos > 29000;
    }

    public ArmDirection getArmDirection(final double armPos){
        return armPos > 180 ? ArmDirection.REVERSE : ArmDirection.FORWARD;
    }

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final int elevatorPos, final double armPos){
        return goToPosition(robot, List.of(), elevatorPos, armPos, List.of());
    }

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final List<Titan.Command<Robot>> preCommands, final int elevatorPos, final double armPos){
        return goToPosition(robot, preCommands, elevatorPos, armPos, List.of());
    }

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final int elevatorPos, final double armPos, final List<Titan.Command<Robot>> extraCommands){
        return goToPosition(robot, List.of(), elevatorPos, armPos, extraCommands);
    }

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final List<Titan.Command<Robot>> preCommands, final int elevatorPos, final double armPos, final List<Titan.Command<Robot>> extraCommands){
        final List<Titan.Command<Robot>> out = new ArrayList<>();
        out.addAll(preCommands);
        final ArmDirection currentDirection = getArmDirection(robot.getArm().getArmAngle());
        final ArmDirection targetDirection = getArmDirection(armPos);
        if(currentDirection != targetDirection){
            if(isArmInStowPosition(robot.getArm().getArmAngle())){
                out.add(new ArmMoveToCommand(140, Constants.AUTO_ARM_SPEED));
            }
            if(isElevatorInStage2(robot.getElevator().getEncoderPosition()) && isElevatorInStage2(elevatorPos)){
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                queue.addCommand(robot, new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                queue.addCommand(robot, new ArmMoveToCommand(armPos, Constants.AUTO_ARM_SPEED));
                out.add(queue);
            }else{
                if(!isElevatorInStage2(robot.getElevator().getEncoderPosition())){
                    out.add(new ElevateToCommand(17000, Constants.AUTO_ELEVATOR_SPEED));
                }
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                if(isElevatorInStage2(elevatorPos)){
                    queue.addCommand(robot, new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                }
                if(isArmInStowPosition(armPos)){
                    queue.addCommand(robot, new ArmMoveToCommand(140, Constants.AUTO_ARM_SPEED));
                }else{
                    queue.addCommand(robot, new ArmMoveToCommand(armPos, Constants.AUTO_ARM_SPEED));
                }
                out.add(queue);
                if(!isElevatorInStage2(elevatorPos)){
                    out.add(new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                }
            }
        }else /* if target is in the same direction as current*/{

            if((robot.getElevator().getEncoderPosition() > 0 && isArmInStowPosition(armPos) && isArmInStowPosition(robot.getArm().getArmAngle())) || (elevatorPos > 0 && isArmInStowPosition(robot.getArm().getArmAngle()))){
                out.add(new ArmMoveToCommand(140, Constants.AUTO_ARM_SPEED));
            }

            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            if(elevatorPos < 3000 && armPos < 100){
                queue.addCommand(robot, new ElevateToCommand(3000, Constants.AUTO_ELEVATOR_SPEED));
            }else{
                queue.addCommand(robot, new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
            }
            if(!isArmInStowPosition(armPos)){
                queue.addCommand(robot, new ArmMoveToCommand(armPos, Constants.AUTO_ARM_SPEED));
            }else{
                queue.addCommand(robot, new ArmMoveToCommand(140, Constants.AUTO_ARM_SPEED));
            }
            out.add(queue);

            if(elevatorPos < 3000 && armPos < 100){
                out.add(new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
            }
        }

        if(isArmInStowPosition(armPos)){
            out.add(new ArmMoveToCommand(armPos, Constants.AUTO_ARM_SPEED));
        }

        out.addAll(extraCommands);

        return out;
    }

    @Override
    public void init(final Robot robot){
        final Supplier<List<Titan.Command<Robot>>> stow =()->goToPosition(robot, 0, Constants.ARM_STOW_ANGLE);

        hatchSequences.put(Sequence.STOW, stow);
        ballSequences.put(Sequence.STOW, stow);

        hatchSequences.put(Sequence.INTAKE, ()->{
            if(robot.getElevator().getEncoderPosition() < 1000 && robot.getArm().getArmAngle() < 100){
                return List.of(new RollerCommand(-Constants.INTAKE_ROLLER_SPEED, -1));
            }else{
                return List.of(new FingerCommand(true), new Titan.WaitCommand<>(400), new JayCommand(true));
            }
            //if(System.currentTimeMillis() > lastIntake + 1000){
            //}else{
            //    return List.of(new JayCommand(false), new FingerCommand(false));
            //}
        });
        hatchSequences.put(Sequence.OUTTAKE, ()->{
            return List.of(new JayCommand(true), new FingerCommand(false));
        });

        ballSequences.put(Sequence.INTAKE, ()->{
            //if(robot.getIntake().isRolling()){
            //    return List.of(new RollerCommand(0.0, -1));
            //}else{
                return List.of(new RollerCommand(Constants.INTAKE_ROLLER_SPEED, -1));
            //}
        });
        ballSequences.put(Sequence.OUTTAKE, ()->{
            //if(robot.getIntake().isRolling()){
            //    return List.of(new RollerCommand(0.0, -1));
            //}else{
                return List.of(new RollerCommand(-Constants.INTAKE_ROLLER_SPEED, -1));
            //}
        });

        hatchSequences.put(Sequence.FLOOR, ()->{
            if(robot.getElevator().getEncoderPosition() < 1000 && robot.getArm().getArmAngle() < 100){
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                queue.addCommand(robot, new RollerCommand(-1.0, 250));
                queue.addCommand(robot, new HatchOuttakeCommand(true));
                return goToPosition(robot, 8000, 90, List.of(new FingerCommand(false), new JayCommand(false), new HatchOuttakeCommand(false), new Titan.WaitCommand<>(400), new FingerCommand(true), new RollerCommand(-1.0, 400), new Titan.WaitCommand<>(200), queue, new JayCommand(true)));
            }else{
                return goToPosition(robot, List.of(new HatchOuttakeCommand(true)), 0, 90);
            }
        });

        ballSequences.put(Sequence.FLOOR, ()->goToPosition(robot, List.of(new RollerCommand(Constants.INTAKE_ROLLER_SPEED, -1)), 0, 90));

        hatchSequences.put(Sequence.LOADING_STATION, ()->goToPosition(robot, 8000, 90, List.of(new JayCommand(false), new FingerCommand(false))));

        hatchSequences.put(Sequence.ROCKET_FORWARD_1, ()->goToPosition(robot, 8000, 90, List.of(new JayCommand(true), new FingerCommand(true))));
        hatchSequences.put(Sequence.ROCKET_FORWARD_2, ()->goToPosition(robot, 31000, 90, List.of(new JayCommand(true), new FingerCommand(true))));

        hatchSequences.put(Sequence.CARGO_SHIP, hatchSequences.get(Sequence.ROCKET_FORWARD_1));

        // BUTTON MAPPINGS
        // LOGITECH
        // sequences.put(Button.TRIGGER.ordinal(), Sequence.STOW);

        // sequences.put(Button.FIVE.ordinal(), Sequence.FLOOR);
        // sequences.put(Button.THREE.ordinal(), Sequence.LOADING_STATION);
        // sequences.put(Button.SIX.ordinal(), Sequence.CARGO_SHIP);

        // sequences.put(Button.TWELVE.ordinal(), Sequence.ROCKET_FORWARD_1);
        // sequences.put(Button.TEN.ordinal(), Sequence.ROCKET_FORWARD_2);
        // sequences.put(Button.EIGHT.ordinal(), Sequence.ROCKET_FORWARD_3);

        // sequences.put(Button.ELEVEN.ordinal(), Sequence.ROCKET_REVERSE_1);
        // sequences.put(Button.NINE.ordinal(), Sequence.ROCKET_REVERSE_2);
        // sequences.put(Button.SEVEN.ordinal(), Sequence.ROCKET_REVERSE_3);

        // sequences.put(Button.TWO.ordinal(), Sequence.CLIMB);

        //BUTTON BOARD

        //nine is outtake
        //fourteen is intake
        sequences.put(13, Sequence.STOW);

        sequences.put(14, Sequence.INTAKE);
        sequences.put(9, Sequence.OUTTAKE);

        sequences.put(11, Sequence.FLOOR);
        sequences.put(8, Sequence.LOADING_STATION);
        sequences.put(6, Sequence.CARGO_SHIP);

        sequences.put(12, Sequence.ROCKET_FORWARD_1);
        sequences.put(7, Sequence.ROCKET_FORWARD_2);
        sequences.put(2, Sequence.ROCKET_FORWARD_3);

        sequences.put(4, Sequence.ROCKET_REVERSE_1);
        sequences.put(3, Sequence.ROCKET_REVERSE_2);
        sequences.put(1, Sequence.ROCKET_REVERSE_3);

        sequences.put(5, Sequence.CLIMB);
        //switch is 16

        if(robot.getMode() == Robot.Mode.AUTO){
            mimicCommands.addAll(loadMimicFile(robot, "TEST"));
        }else if(robot.getMode() == Robot.Mode.TEST){
            robot.getDrivebase().setHome();
            observer.prepare("TEST");
        }

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
        return buttonBoard.getRawButton(16) ? SequenceType.HATCH : SequenceType.CARGO;
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