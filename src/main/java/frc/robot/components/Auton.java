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

    public List<Titan.Command<Robot>> generateSequence(final Robot robot, final IntakePosition pos, final List<Titan.Command<Robot>> in){
        final List<Titan.Command<Robot>> output = new ArrayList<>();
        final IntakePosition currentPosition = robot.getArm().getWristPosition() > 180 ? IntakePosition.REVERSE : IntakePosition.FORWARD;
        if(currentPosition != pos){
            final Titan.ParallelCommandGroup<Robot> queue = new Titan.ParallelCommandGroup<>();
            queue.addCommand(robot, new WristCommand(true));
            queue.addCommand(robot, new FingerCommand(true));
            queue.addCommand(robot, new HatchOuttakeCommand(true));
            output.add(queue);
            if(Titan.approxEquals(robot.getArm().getWristPosition(), 175, 5)){
                output.add(new ArmMoveToCommand(160, Constants.AUTO_ROBOT_ARM_SPEED));
                output.add(new CarriageUpCommand(Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            }else{
                output.add(new CarriageUpCommand(Constants.AUTO_ROBOT_ELEVATOR_SPEED));
                output.add(new ArmMoveToCommand(150, Constants.AUTO_ROBOT_ARM_SPEED));    
            }
        }
        output.addAll(in);
        return output;
    }

    public List<Titan.Command<Robot>> getBallRocketSequence(final Robot robot, final IntakePosition pos, final double height){
        final Titan.ParallelCommandGroup<Robot> stowQueue = new Titan.ParallelCommandGroup<>();
        if(pos == IntakePosition.FORWARD){
            stowQueue.addCommand(robot, new WristCommand(true));
            stowQueue.addCommand(robot, new FingerCommand(true));
            stowQueue.addCommand(robot, new HatchOuttakeCommand(true));
            stowQueue.addCommand(robot, new ArmMoveToCommand(128, Constants.AUTO_ROBOT_ARM_SPEED));
            stowQueue.addCommand(robot, new ElevateToCommand(height, Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            return generateSequence(robot, IntakePosition.FORWARD, List.of(stowQueue));
        }else{
            stowQueue.addCommand(robot, new FingerCommand(true));
            stowQueue.addCommand(robot, new HatchOuttakeCommand(true));
            final Titan.CommandQueue<Robot> subRoutine = new Titan.CommandQueue<>();
            if(!Titan.approxEquals(robot.getArm().getWristPosition(), 130, 20)){
                subRoutine.add(new ArmMoveToCommand(130, Constants.AUTO_ROBOT_ARM_SPEED));
            }
            subRoutine.add(new WristCommand(false));
            stowQueue.addQueue(robot, subRoutine);
            return generateSequence(robot, IntakePosition.REVERSE, List.of(stowQueue, new ElevateToCommand(height, Constants.AUTO_ROBOT_ELEVATOR_SPEED), new ArmMoveToCommand(174, Constants.AUTO_ROBOT_ARM_SPEED)));
        }
    }

    public void init(final Robot robot){

        //STOWING
        sequences.put(Button.TRIGGER, ()->{
            final Titan.ParallelCommandGroup<Robot> stowQueue = new Titan.ParallelCommandGroup<>();
            stowQueue.addCommand(robot, new WristCommand(true));
            stowQueue.addCommand(robot, new FingerCommand(true));
            stowQueue.addCommand(robot, new HatchOuttakeCommand(true));
            if(robot.getElevator().getEncoderPosition() > 200){
                stowQueue.addCommand(robot, new ArmMoveToCommand(150, Constants.AUTO_ROBOT_ARM_SPEED));
                stowQueue.addCommand(robot, new ElevateToCommand(0, Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            }
            return generateSequence(robot, IntakePosition.FORWARD, List.of(stowQueue, new ArmMoveToCommand(176, 0.5)));
        });

        //FLOOR PICKUP
        sequences.put(Button.FIVE, ()->{
            final Titan.ParallelCommandGroup<Robot> stowQueue = new Titan.ParallelCommandGroup<>();
            stowQueue.addCommand(robot, new WristCommand(true));
            stowQueue.addCommand(robot, new FingerCommand(true));
            stowQueue.addCommand(robot, new HatchOuttakeCommand(true));
            stowQueue.addCommand(robot, new ArmMoveToCommand(90, Constants.AUTO_ROBOT_ARM_SPEED));
            stowQueue.addCommand(robot, new ElevateToCommand(0, Constants.AUTO_ROBOT_ELEVATOR_SPEED));
            return generateSequence(robot, IntakePosition.FORWARD, List.of(stowQueue));
        });

        sequences.put(Button.TWELVE, ()->getBallRocketSequence(robot, IntakePosition.FORWARD, 0.0));
        sequences.put(Button.TEN, ()->getBallRocketSequence(robot, IntakePosition.FORWARD, 26000));
        sequences.put(Button.EIGHT, ()->getBallRocketSequence(robot, IntakePosition.FORWARD, 50000));

        sequences.put(Button.NINE, ()-> getBallRocketSequence(robot, IntakePosition.REVERSE, 16500));
        sequences.put(Button.SEVEN, ()-> getBallRocketSequence(robot, IntakePosition.REVERSE, 38500));

        commands.init(robot);
    }

    public void periodic(final Robot robot){
        if(buttonBoard.getRawButton(Button.TRIGGER)){
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
}