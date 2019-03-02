package frc.robot.components;

import java.util.Map;
import java.util.EnumMap;
import java.util.List;
import java.util.ArrayList;

import java.util.function.Supplier;

import frc.robot.Constants;
import frc.robot.ControlMode;
import frc.robot.MimicPropertyValue;
import frc.robot.Robot;
import frc.robot.Titan;
import frc.robot.Titan.LogitechExtreme3D.Axis;
import frc.robot.Titan.LogitechExtreme3D.Button;

import frc.robot.commands.ElevateToCommand;
import frc.robot.commands.ArmMoveToCommand;
import frc.robot.commands.FingerCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.HatchOuttakeCommand;
import frc.robot.commands.MimicCommand;
import frc.robot.commands.CarriageUpCommand;

public class Auton {
    private static enum Sequence {
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
        CLIMB
    }

    private static enum ArmDirection{
        FORWARD, REVERSE
    };

    private Titan.CommandQueue<Robot> commands;
    private Titan.LogitechExtreme3D buttonBoard;

    private EnumMap<Sequence, Supplier<List<Titan.Command<Robot>>>> hatchSequences = new EnumMap<>(Sequence.class);
    private EnumMap<Sequence, Supplier<List<Titan.Command<Robot>>>> ballSequences = new EnumMap<>(Sequence.class);
    private EnumMap<Titan.LogitechExtreme3D.Button, Sequence> sequences = new EnumMap<>(Titan.LogitechExtreme3D.Button.class);

    private boolean isControllingRoller = false, isControllingHatch = false;

    private Titan.Mimic.Observer<Robot, MimicPropertyValue> observer;

    public Auton(){
        commands = new Titan.CommandQueue<>();
        buttonBoard = new Titan.LogitechExtreme3D(Constants.BUTTONBOARD_JOYSTICK_ID);

        observer = new Titan.Mimic.Observer<>();
    }

    public boolean isArmInStowPosition(final double armPos){
        return Titan.approxEquals(armPos, 175, 5);
    }

    public boolean isElevatorInStage2(final int elevatorPos){
        return elevatorPos > 29000;
    }

    public ArmDirection getArmDirection(final double armPos){
        return armPos > 180 ? ArmDirection.REVERSE : ArmDirection.FORWARD;
    }

    public List<Titan.Command<Robot>> goToPosition(final Robot robot, final int elevatorPos, final double armPos){
        final List<Titan.Command<Robot>> out = new ArrayList<>();
        final ArmDirection currentDirection = getArmDirection(robot.getArm().getArmAngle());
        final ArmDirection targetDirection = getArmDirection(armPos);
        if(currentDirection != targetDirection){
            if(isArmInStowPosition(robot.getArm().getArmAngle())){
                out.add(new ArmMoveToCommand(160, Constants.AUTO_ARM_SPEED));
            }
            if(isElevatorInStage2(robot.getElevator().getEncoderPosition()) && isElevatorInStage2(elevatorPos)){
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                queue.addCommand(robot, new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                queue.addCommand(robot, new ArmMoveToCommand(armPos, Constants.AUTO_ARM_SPEED));
                out.add(queue);
            }else{
                if(!isElevatorInStage2(robot.getElevator().getEncoderPosition())){
                    out.add(new CarriageUpCommand(Constants.AUTO_ELEVATOR_SPEED + 0.1));
                }
                final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
                if(isElevatorInStage2(elevatorPos)){
                    queue.addCommand(robot, new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                }
                if(isArmInStowPosition(armPos)){
                    queue.addCommand(robot, new ArmMoveToCommand(160, Constants.AUTO_ARM_SPEED));
                }else{
                    queue.addCommand(robot, new ArmMoveToCommand(armPos, Constants.AUTO_ARM_SPEED));
                }
                out.add(queue);
                if(!isElevatorInStage2(elevatorPos)){
                    out.add(new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
                }
            }
        }else /* if target is in the same direction as current*/{

            if(isArmInStowPosition(armPos) || (elevatorPos > 0 && isArmInStowPosition(robot.getArm().getArmAngle()))){
                out.add(new ArmMoveToCommand(160, Constants.AUTO_ARM_SPEED));
            }

            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            if(elevatorPos < 3000 && armPos < 100){
                queue.addCommand(robot, new ElevateToCommand(3000, Constants.AUTO_ELEVATOR_SPEED));
            }else{
                queue.addCommand(robot, new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
            }
            if(!isArmInStowPosition(armPos)){
                queue.addCommand(robot, new ArmMoveToCommand(armPos, Constants.AUTO_ARM_SPEED));
            }
            out.add(queue);
            
            if(elevatorPos < 3000 && armPos < 100){
                queue.addCommand(robot, new ElevateToCommand(elevatorPos, Constants.AUTO_ELEVATOR_SPEED));
            }
        }

        if(isArmInStowPosition(armPos)){
            out.add(new ArmMoveToCommand(armPos, Constants.AUTO_ARM_SPEED));
        }

        return out;
    }

    public List<Titan.Command<Robot>> generateSequence(final Robot robot, final ArmDirection pos, final Supplier<List<Titan.Command<Robot>>> in){
        final List<Titan.Command<Robot>> output = new ArrayList<>();
        final ArmDirection currentPosition = robot.getArm().getArmAngle() > 180 ? ArmDirection.REVERSE : ArmDirection.FORWARD;
        if(currentPosition != pos){
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            output.add(queue);
            if(Titan.approxEquals(robot.getArm().getArmAngle(), 175, 5)){
                output.add(new ArmMoveToCommand(160, Constants.AUTO_ARM_SPEED));
                output.add(new CarriageUpCommand(Constants.AUTO_ELEVATOR_SPEED + 0.1));
            }else{
                final Titan.ParallelCommandGroup<Robot> queue2 = new Titan.ParallelCommandGroup<>();
                queue2.addCommand(robot, new ArmMoveToCommand(currentPosition == ArmDirection.FORWARD ? 160 : 255, Constants.AUTO_ARM_SPEED));
                queue2.addCommand(robot, new WristCommand(true));
                output.add(queue2);
                output.add(new CarriageUpCommand(Constants.AUTO_ELEVATOR_SPEED + 0.1));
            }
            output.add(new ArmMoveToCommand(pos == ArmDirection.FORWARD ? 150 : 255, Constants.AUTO_ARM_SPEED));    
        }
        output.add(new Titan.ConsumerCommand<Robot>((rob)->{
            commands.addAll(in.get());
        }));
        return output;
    }

    public List<Titan.Command<Robot>> getBallRocketSequence(final Robot robot, final double height){
        return generateSequence(robot, ArmDirection.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new WristCommand(true));
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            queue.addCommand(robot, new ArmMoveToCommand(128, Constants.AUTO_ARM_SPEED));
            queue.addCommand(robot, new ElevateToCommand(height, Constants.AUTO_ELEVATOR_SPEED));
            return List.of(queue);
        });
    }

    public List<Titan.Command<Robot>> getReverseBallRocketSequence(final Robot robot, final double height){
        return generateSequence(robot, ArmDirection.FORWARD, ()-> {
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            final Titan.CommandQueue<Robot> subRoutine = new Titan.CommandQueue<>();
            if(!Titan.approxEquals(robot.getArm().getArmAngle(), 130, 10) && robot.getArm().isWristing()){
                subRoutine.add(new ArmMoveToCommand(130, Constants.AUTO_ARM_SPEED));
            }
            subRoutine.add(new WristCommand(false));
            queue.addQueue(robot, subRoutine);
            return List.of(queue, new ElevateToCommand(height, Constants.AUTO_ELEVATOR_SPEED), new ArmMoveToCommand(178, Constants.AUTO_ARM_SPEED));
        });
    }

    public List<Titan.Command<Robot>> getHatchRocketSequence(final Robot robot, final double height){
        return generateSequence(robot, ArmDirection.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue1 = new Titan.ParallelCommandGroup<>();
            final List<Titan.Command<Robot>> commands = new ArrayList<>();
            queue1.addCommand(robot, new FingerCommand(true));
            queue1.addCommand(robot, new HatchOuttakeCommand(true));
            commands.add(queue1);
            if(robot.getArm().getArmAngle() > 140){
                commands.add(new ArmMoveToCommand(140, Constants.AUTO_ARM_SPEED));
            }
            final Titan.ParallelCommandGroup<Robot> queue2  = new Titan.ParallelCommandGroup<>();
            queue2.addCommand(robot, new WristCommand(false));
            //pls fix arm compensation code
            queue2.addCommand(robot, new ArmMoveToCommand(95, Constants.AUTO_ARM_SPEED));
            queue2.addCommand(robot, new ElevateToCommand(height, Constants.AUTO_ELEVATOR_SPEED));
            commands.add(queue2);
            return commands;
        });
    }

    public List<Titan.Command<Robot>> getReverseHatchRocketSequence(final Robot robot, final double height){
        return generateSequence(robot, ArmDirection.REVERSE, ()->{
            final Titan.ParallelCommandGroup<Robot> queue1 = new Titan.ParallelCommandGroup<>();
            return List.of(queue1);
        });
    }

    public List<Titan.Command<Robot>> getFloorIntake(final Robot robot, final double height){
        return generateSequence(robot, ArmDirection.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            queue.addCommand(robot, new ArmMoveToCommand(87, Constants.AUTO_ARM_SPEED));
            if(robot.getElevator().getEncoderPosition() < 3000){
                queue.addCommand(robot, new ElevateToCommand(3000, Constants.AUTO_ELEVATOR_SPEED));
                return List.of(queue, new WristCommand(true), new ElevateToCommand(height, 0.1));
            }else{
                queue.addCommand(robot, new WristCommand(true));
                queue.addCommand(robot, new ElevateToCommand(height, Constants.AUTO_ELEVATOR_SPEED));
                return List.of(queue);
            }
        });
    }

    public void init(final Robot robot){
        //STOWING
        final Supplier<List<Titan.Command<Robot>>> stowSequence = ()->generateSequence(robot, ArmDirection.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> stowQueue = new Titan.ParallelCommandGroup<>();
            stowQueue.addCommand(robot, new WristCommand(true));
            stowQueue.addCommand(robot, new FingerCommand(true));
            stowQueue.addCommand(robot, new HatchOuttakeCommand(true));
            if(robot.getElevator().getEncoderPosition() > 200){
                stowQueue.addCommand(robot, new ArmMoveToCommand(150, Constants.AUTO_ARM_SPEED));
                stowQueue.addCommand(robot, new ElevateToCommand(0, Constants.AUTO_ELEVATOR_SPEED));
            }
            return List.of(stowQueue, new ArmMoveToCommand(180, 0.3));
        });


        hatchSequences.put(Sequence.STOW, stowSequence);
        ballSequences.put(Sequence.STOW, stowSequence);

        //FLOOR PICKUP
        hatchSequences.put(Sequence.FLOOR, ()-> getFloorIntake(robot, 1000));
        ballSequences.put(Sequence.FLOOR, ()-> getFloorIntake(robot, 0));

        final Supplier<List<Titan.Command<Robot>>> hatch1 = ()-> getHatchRocketSequence(robot, 0);

        // FORWARD HATCH ROCKET 
        hatchSequences.put(Sequence.ROCKET_FORWARD_1, hatch1);
        hatchSequences.put(Sequence.ROCKET_FORWARD_2, ()-> getHatchRocketSequence(robot, 24000));
        hatchSequences.put(Sequence.ROCKET_FORWARD_3, ()-> getHatchRocketSequence(robot, 46000));

        // FORWARD BALL ROCKET
        ballSequences.put(Sequence.ROCKET_FORWARD_1, ()->getBallRocketSequence(robot, 0));
        ballSequences.put(Sequence.ROCKET_FORWARD_2, ()->getBallRocketSequence(robot, 24000));
        ballSequences.put(Sequence.ROCKET_FORWARD_3, ()->getBallRocketSequence(robot, 48000));

        // REVERSE BALL ROCKET
        ballSequences.put(Sequence.ROCKET_REVERSE_2, ()-> getReverseBallRocketSequence(robot, 18000));
        ballSequences.put(Sequence.ROCKET_REVERSE_3, ()-> getReverseBallRocketSequence(robot, 38500));

        //CARGO SHIP
        ballSequences.put(Sequence.CARGO_SHIP, ()->generateSequence(robot, ArmDirection.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new WristCommand(true));
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            queue.addCommand(robot, new ArmMoveToCommand(90, Constants.AUTO_ARM_SPEED));
            queue.addCommand(robot, new ElevateToCommand(28000, Constants.AUTO_ELEVATOR_SPEED));
            return List.of(queue);
        }));
        hatchSequences.put(Sequence.CARGO_SHIP, hatch1);

        // LOADING STATION
        ballSequences.put(Sequence.LOADING_STATION, ()-> getHatchRocketSequence(robot, 11500));
        hatchSequences.put(Sequence.LOADING_STATION, ()-> generateSequence(robot, ArmDirection.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new WristCommand(false));
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            return List.of(queue, new ArmMoveToCommand(120, Constants.AUTO_ARM_SPEED), new Titan.ConsumerCommand<Robot>((rob)->{
                commands.addAll(stowSequence.get());
            }));
        }));

        final Supplier<List<Titan.Command<Robot>>> climb = ()->generateSequence(robot, ArmDirection.REVERSE, ()->{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new WristCommand(true));
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            return List.of(queue, new ElevateToCommand(12000, Constants.AUTO_ELEVATOR_SPEED), new ArmMoveToCommand(255, Constants.AUTO_ARM_SPEED));
        });
        ballSequences.put(Sequence.CLIMB, climb);
        hatchSequences.put(Sequence.CLIMB, climb);

        // BUTTON MAPPINGS
        sequences.put(Button.TRIGGER, Sequence.STOW);

        sequences.put(Button.FIVE, Sequence.FLOOR);
        sequences.put(Button.THREE, Sequence.LOADING_STATION);
        sequences.put(Button.SIX, Sequence.CARGO_SHIP);

        sequences.put(Button.TWELVE, Sequence.ROCKET_FORWARD_1);
        sequences.put(Button.TEN, Sequence.ROCKET_FORWARD_2);
        sequences.put(Button.EIGHT, Sequence.ROCKET_FORWARD_3);

        sequences.put(Button.ELEVEN, Sequence.ROCKET_REVERSE_1);
        sequences.put(Button.NINE, Sequence.ROCKET_REVERSE_2);
        sequences.put(Button.SEVEN, Sequence.ROCKET_REVERSE_3);

        sequences.put(Button.TWO, Sequence.CLIMB);

        if(robot.getMode() == Robot.Mode.AUTO){
            commands.add(new MimicCommand(robot, "TEST"));
            //commands.add(new DriveCommand(-0.3, -0.3, 115, 85, 12));
        }else if(robot.getMode() == Robot.Mode.TEST){
            robot.getDrivebase().setHome();
            observer.prepare("TEST");
        }

        commands.init(robot);
    }

    public void periodic(final Robot robot){
        final boolean isHatch = buttonBoard.getRawAxis(Axis.SLIDER) > 0;

        if(buttonBoard.getRawButton(Button.TRIGGER)){
            commands.clear();
            robot.getArm().setControlMode(ControlMode.MANUAL);
            robot.getElevator().setControlMode(ControlMode.MANUAL);
            isControllingHatch = false;
            isControllingRoller = false;
        }

        if(commands.isEmpty()){
            for(final Map.Entry<Button, Sequence> e : sequences.entrySet()){
                if(buttonBoard.getRawButton(e.getKey())){
                    final Sequence seq = e.getValue();
                    final Supplier<List<Titan.Command<Robot>>> selected = (isHatch ? hatchSequences : ballSequences).getOrDefault(seq, ()->List.of());
                    commands.addAll(selected.get());
                    commands.init(robot);
                    break;
                }
            }
        }

        if(buttonBoard.getRawButton(Button.FOUR)){
            if(isHatch){
                isControllingHatch = true;
                robot.getIntake().actuateHatch(false);
            }
        }else if(buttonBoard.getPOV() == 0){
            if(isHatch){
                isControllingHatch = true;
                //if(robot.getIntake().getHatchDistance() < 30){
                robot.getIntake().actuateHatch(false);
                //}
            } else{
                isControllingRoller = true;
                robot.getIntake().roll(-Constants.INTAKE_ROLLER_SPEED);
            }
            //up
        }else if(buttonBoard.getPOV() == 180){
            if(isHatch){
                isControllingHatch = true;
                robot.getIntake().finger(false);
            } else{
                isControllingRoller = true;
                robot.getIntake().roll(Constants.INTAKE_ROLLER_SPEED);
            }
            //down
        }else if(isControllingRoller){
            isControllingRoller = false;
            robot.getIntake().roll(0.0);
        }else if(isControllingHatch){
            isControllingHatch = false;
            robot.getIntake().actuateHatch(true);
            robot.getIntake().finger(true);
        }

        // if(buttonBoard.getRawButton(Button.TWO)){
        //     commands.done(robot);
        // }

        commands.update(robot);

        if(robot.getMode() == Robot.Mode.TEST){
            observer.addStep(robot, MimicPropertyValue.class);
        }
    }

    public void disabled(final Robot robot){
        commands.done(robot);

        observer.save();
    }

    public Titan.CommandQueue<Robot> getCommands(){
        return commands;
    }
}