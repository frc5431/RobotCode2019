package frc.robot.components;

import java.util.Map;
import java.util.EnumMap;
import java.util.List;
import java.util.ArrayList;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Titan;
import frc.robot.Titan.LogitechExtreme3D.Button;

import frc.robot.commands.ElevateToCommand;
import frc.robot.commands.ArmMoveToCommand;
import frc.robot.commands.FingerCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.HatchOuttakeCommand;
import frc.robot.commands.CarriageUpCommand;

public class Auton {
    private static enum IntakePosition{
        FORWARD, REVERSE
    };

    private Titan.CommandQueue<Robot> commands;
    private Titan.LogitechExtreme3D buttonBoard;

    private EnumMap<Titan.LogitechExtreme3D.Button, Supplier<List<Titan.Command<Robot>>>> sequences = new EnumMap<>(Titan.LogitechExtreme3D.Button.class);

    public Auton(){
        commands = new Titan.CommandQueue<>();
        buttonBoard = new Titan.LogitechExtreme3D(Constants.BUTTONBOARD_JOYSTICK_ID);
    }

    public List<Titan.Command<Robot>> generateSequence(final Robot robot, final IntakePosition pos, final Supplier<List<Titan.Command<Robot>>> in){
        final List<Titan.Command<Robot>> output = new ArrayList<>();
        final IntakePosition currentPosition = robot.getArm().getWristPosition() > 180 ? IntakePosition.REVERSE : IntakePosition.FORWARD;
        if(currentPosition != pos){
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            output.add(queue);
            if(Titan.approxEquals(robot.getArm().getWristPosition(), 175, 5)){
                output.add(new ArmMoveToCommand(160, Constants.AUTO_ROBOT_ARM_SPEED));
                output.add(new CarriageUpCommand(Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            }else{
                final Titan.ParallelCommandGroup<Robot> queue2 = new Titan.ParallelCommandGroup<>();
                queue2.addCommand(robot, new ArmMoveToCommand(250, Constants.AUTO_ROBOT_ARM_SPEED));
                queue2.addCommand(robot, new WristCommand(true));
                output.add(queue2);
                output.add(new CarriageUpCommand(Constants.AUTO_ROBOT_ELEVATOR_SPEED));
                output.add(new ArmMoveToCommand(150, Constants.AUTO_ROBOT_ARM_SPEED));    
            }
        }
        output.add(new Titan.ConsumerCommand<Robot>((rob)->{
            commands.addAll(in.get());
        }));
        return output;
    }

    public List<Titan.Command<Robot>> getBallRocketSequence(final Robot robot, final double height){
        return generateSequence(robot, IntakePosition.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new WristCommand(true));
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            queue.addCommand(robot, new ArmMoveToCommand(128, Constants.AUTO_ROBOT_ARM_SPEED));
            queue.addCommand(robot, new ElevateToCommand(height, Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            return List.of(queue);
        });
    }

    public List<Titan.Command<Robot>> getReverseBallRocketSequence(final Robot robot, final double height){
        return generateSequence(robot, IntakePosition.FORWARD, ()-> {
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            final Titan.CommandQueue<Robot> subRoutine = new Titan.CommandQueue<>();
            if(!Titan.approxEquals(robot.getArm().getWristPosition(), 130, 10) && robot.getArm().isWristing()){
                subRoutine.add(new ArmMoveToCommand(130, Constants.AUTO_ROBOT_ARM_SPEED));
            }
            subRoutine.add(new WristCommand(false));
            queue.addQueue(robot, subRoutine);
            return List.of(queue, new ElevateToCommand(height, Constants.AUTO_ROBOT_ELEVATOR_SPEED), new ArmMoveToCommand(174, Constants.AUTO_ROBOT_ARM_SPEED));
        });
    }

    public List<Titan.Command<Robot>> getHatchRocketSequence(final Robot robot, final double height){
        return generateSequence(robot, IntakePosition.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue1 = new Titan.ParallelCommandGroup<>();
            final List<Titan.Command<Robot>> commands = new ArrayList<>();
            queue1.addCommand(robot, new FingerCommand(true));
            queue1.addCommand(robot, new HatchOuttakeCommand(true));
            commands.add(queue1);
            if(robot.getArm().getWristPosition() > 140){
                commands.add(new ArmMoveToCommand(140, Constants.AUTO_ROBOT_ARM_SPEED));
            }
            final Titan.ParallelCommandGroup<Robot> queue2  = new Titan.ParallelCommandGroup<>();
            queue2.addCommand(robot, new WristCommand(false));
            //pls fix arm compensation code
            queue2.addCommand(robot, new ArmMoveToCommand(95, Constants.AUTO_ROBOT_ARM_SPEED));
            queue2.addCommand(robot, new ElevateToCommand(height, Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            commands.add(queue2);
            return commands;
        });
    }

    public List<Titan.Command<Robot>> getReverseHatchRocketSequence(final Robot robot, final double height){
        return generateSequence(robot, IntakePosition.REVERSE, ()->{
            final Titan.ParallelCommandGroup<Robot> queue1 = new Titan.ParallelCommandGroup<>();
            return List.of(queue1);
        });
    }

    public void init(final Robot robot){

        //STOWING
        sequences.put(Button.TRIGGER, ()->generateSequence(robot, IntakePosition.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> stowQueue = new Titan.ParallelCommandGroup<>();
            stowQueue.addCommand(robot, new WristCommand(true));
            stowQueue.addCommand(robot, new FingerCommand(true));
            stowQueue.addCommand(robot, new HatchOuttakeCommand(true));
            if(robot.getElevator().getEncoderPosition() > 200){
                stowQueue.addCommand(robot, new ArmMoveToCommand(150, Constants.AUTO_ROBOT_ARM_SPEED));
                stowQueue.addCommand(robot, new ElevateToCommand(0, Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            }
            return List.of(stowQueue, new ArmMoveToCommand(180, 0.3));
        }));

        //FLOOR PICKUP
        sequences.put(Button.FIVE, ()->generateSequence(robot, IntakePosition.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new WristCommand(true));
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            queue.addCommand(robot, new ArmMoveToCommand(90, Constants.AUTO_ROBOT_ARM_SPEED));
            queue.addCommand(robot, new ElevateToCommand(0, Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            return List.of(queue);
        }));

        //CARGO SHIP BALL
        sequences.put(Button.SIX, ()->generateSequence(robot, IntakePosition.FORWARD, ()->{
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new WristCommand(true));
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            queue.addCommand(robot, new ArmMoveToCommand(90, Constants.AUTO_ROBOT_ARM_SPEED));
            queue.addCommand(robot, new ElevateToCommand(28000, Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            return List.of(queue);
        }));

        // FORWARD BALL ROCKET
        sequences.put(Button.TWELVE, ()->getBallRocketSequence(robot, 0.0));
        sequences.put(Button.TEN, ()->getBallRocketSequence(robot, 24000));
        sequences.put(Button.EIGHT, ()->getBallRocketSequence(robot, 48000));

        // sequences.put(Button.NINE, ()-> getReverseBallRocketSequence(robot, 16500));
        // sequences.put(Button.SEVEN, ()-> getReverseBallRocketSequence(robot, 38500));
        
        // FORWARD HATCH ROCKET 
        //1
        sequences.put(Button.ELEVEN, ()-> getHatchRocketSequence(robot, 5800));
        //2
        sequences.put(Button.NINE, ()-> getHatchRocketSequence(robot, 28000));
        //3
        sequences.put(Button.SEVEN, ()-> getHatchRocketSequence(robot, 50000));


        // LOADING STATION BALL
        sequences.put(Button.THREE, ()-> getHatchRocketSequence(robot, 11500));

        commands.init(robot);
    }

    public void periodic(final Robot robot){
        if(buttonBoard.getRawButton(Button.TRIGGER) || buttonBoard.getRawButton(Button.TWO)){
            commands.done(robot);
        }

        if(commands.isEmpty()){
            for(final Map.Entry<Button, Supplier<List<Titan.Command<Robot>>>> e : sequences.entrySet()){
                if(buttonBoard.getRawButton(e.getKey())){
                    commands.addAll(e.getValue().get());
                    commands.init(robot);
                    break;
                }
            }
        }

        commands.update(robot);
    }

    public void disabled(final Robot robot){
        commands.done(robot);
    }

    public Titan.CommandQueue<Robot> getCommands(){
        return commands;
    }
}