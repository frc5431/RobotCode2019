package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.auto.commands.MimicDriveCommand;
import frc.robot.auto.commands.DriveToCommand;
import frc.robot.auto.commands.DriveToArcCommand;
import frc.robot.auto.commands.TurnCommand;
import frc.robot.auto.commands.WaitForTargetCommand;
import frc.robot.components.Vision.LEDState;
import frc.robot.util.ControlMode;
import frc.team5431.titan.core.mimic.Mimic;
import frc.team5431.titan.core.mimic.Step;

public enum Path {
    HAB1_TO_FROCKET_GEN((sequence, swapped)->{
        final ParallelCommandGroup group = new ParallelCommandGroup();
        final DriveToArcCommand arc = new DriveToArcCommand(-241, -0.8, 35 * swapped);
        group.addCommands(arc, new TurnCommand(-(90-61.25) * swapped));
        group.addCommands(new WaitUntilCommand(() -> arc.getProgress(Robot.getRobot().getDrivebase()) >= 0.3),new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()));
        return List.of(group);
    }),
    HAB2_TO_FROCKET_GEN((sequence, swapped)->{
        final List<Command> outCommands = new ArrayList<>();
        outCommands.add(new DriveToArcCommand(-80, -0.7, 0));
        outCommands.add(new TurnCommand(25 * swapped));
        outCommands.add(new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()));
        outCommands.add(new DriveToArcCommand(-110, -0.9, 25 * swapped));
        outCommands.add(new RunCommand(()->{
            Robot.getRobot().getVision().setLEDState(LEDState.ON);
        }, Robot.getRobot().getDrivebase()));
        outCommands.add(new TurnCommand(-(90-61.25) * swapped));
        outCommands.add(new WaitForTargetCommand());
        // outCommands.add(new DriveToCommand(-40, -0.5, AutoType.DRIVE_TO));
        // outCommands.add(new DriveToArcCommand(-70, -0.5, 20 * swapped, 1.0));
        // final ParallelCommandGroup<Robot> group = new ParallelCommandGroup<>();
        // final DriveToArcCommand arc = new DriveToArcCommand(-120, -0.8, 50 * swapped);
        // // group.addQueue(List.of(arc, new TurnCommand(-(90-61.25) * swapped)));
        // // group.addQueue(List.of(new ConditionalCommand<>((rob)->arc.getProgress(Robot.getRobot().getDrivebase()) >= 0.3),new RunCommand(()->{
        // //     Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        // // }, Robot.getRobot().getDrivebase())));
        // //outCommands.add(arc);
        // outCommands.add(group);
        return outCommands;
    }),
    FROCKET_TO_LS_GEN((sequence, swapped)->List.of(
        new DriveToCommand(-15, -0.5),
        new TurnCommand(15 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()),
        new DriveToArcCommand(90, 0.7, 15 * swapped),
        new TurnCommand(-20 * swapped),
        new DriveToArcCommand(70, 0.6, -20 * swapped),
        new TurnCommand(0)
    )),
    LS_TO_CROCKET_GEN((sequence, swapped)->List.of(
        new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()),
        new DriveToArcCommand(-110, -0.7, 0),
        new TurnCommand((180 - 61.25) * swapped)
    )),
    HAB2_TO_SCARGO_GEN((sequence, swapped)->List.of(
        new DriveToArcCommand(-100, -0.7, 0),
        new TurnCommand(20 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()),
        new DriveToArcCommand(swapped == -1 ? -66 : -61, -0.9, 20 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getVision().setLEDState(LEDState.ON);
        }, Robot.getRobot().getDrivebase()),
        new TurnCommand(90 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()),
        new WaitForTargetCommand(),
        new DriveToArcCommand(8, 0.2, 90 * swapped)
    )),
    SCARGO_TO_LS_GEN((sequence, swapped)->List.of(
        new DriveToArcCommand(-5, -0.9, 90 * swapped),
        new TurnCommand(-30 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()),
        new DriveToArcCommand(53, 0.9, -30* swapped),
        new RunCommand(()->{
            Robot.getRobot().getVision().setLEDState(LEDState.ON);
        }, Robot.getRobot().getDrivebase()),
        new TurnCommand(0)
    )),
    LS_TO_SCARGO_GEN((sequence, swapped)->List.of(
        new DriveToArcCommand(-40, -0.4, 0),
        new TurnCommand(-10 * swapped),
        new DriveToArcCommand(-185, -0.9, -10 * swapped),
        //new TurnCommand(0 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()),
        //new DriveToArcCommand(-100, -0.5, 0 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getVision().setLEDState(LEDState.ON);
        }, Robot.getRobot().getDrivebase()),
        new TurnCommand(90 * swapped),
        new WaitForTargetCommand()
    )),
    HAB_TO_CCARGO_GEN((sequence, swapped)->List.of(
        new DriveToCommand(70, 0.5),
        new RunCommand(()->{
            //Sequence.values()[stepSequence];
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, Sequence.ROCKET_FORWARD_1);
        })
    )),
    CCARGO_TO_LS_GEN((sequence, swapped)->List.of(
        new DriveToCommand(-20, -0.2),
        new TurnCommand(-120 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()),
        new DriveToArcCommand(95, 0.7, -120 * swapped),
        new TurnCommand(-180 * swapped)
    )),
    LS_TO_CCARGO_GEN((sequence, swapped)->List.of(
        new DriveToArcCommand(-70, -0.7, -180 * swapped),
        new TurnCommand(-320 * swapped),
        new RunCommand(()->{
            Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.HATCH, sequence);
        }, Robot.getRobot().getDrivebase()),
        new DriveToArcCommand(60, 0.7, -320 * swapped),
        new TurnCommand(0)
    )),
    TEST(mimicGenerator("TEST"));

    private static BiFunction<Sequence, Double, List<Command>> mimicGenerator(final String name){
        return (sequence, swapped)->{
            final List<Command> outCommands = new ArrayList<>();

            int lastRunningSequence = -1;

            //Collect the mimic file
            //mimicChooser.getSelected()
            final List<Step<MimicPropertyValue>> steps = Mimic.load(name, MimicPropertyValue.class);
            for(final Step<MimicPropertyValue> step : steps){
                final List<Command> out = new ArrayList<>();
                if(step.getBoolean(MimicPropertyValue.HOME)){
                    out.add(new RunCommand(()->{
                        Robot.getRobot().getDrivebase().reset();
                    }, Robot.getRobot().getDrivebase()));
                }

                final int stepSequence = step.getInteger(MimicPropertyValue.RUNNING_SEQUENCE);
                if(sequence != null && stepSequence >= 0 && stepSequence != lastRunningSequence){
                    out.add(new RunCommand(()->{
                        //Sequence.values()[stepSequence];
                        Robot.getRobot().getAuton().runSequence(Robot.getRobot(), SequenceType.values()[step.getInteger(MimicPropertyValue.SEQUENCE_TYPE)], sequence);
                    }));
                }
                lastRunningSequence = stepSequence;

                final double leftPower, leftDistance;
                final double rightPower, rightDistance;
                final double angle;

                if(swapped == -1){
                    leftPower = step.getDouble(MimicPropertyValue.RIGHT_POWER);
                    rightPower = step.getDouble(MimicPropertyValue.LEFT_POWER);

                    leftDistance = step.getDouble(MimicPropertyValue.RIGHT_DISTANCE);
                    rightDistance = step.getDouble(MimicPropertyValue.LEFT_DISTANCE);

                    angle = -step.getDouble(MimicPropertyValue.ANGLE);
                }else{
                    leftPower = step.getDouble(MimicPropertyValue.LEFT_POWER);
                    rightPower = step.getDouble(MimicPropertyValue.RIGHT_POWER);

                    leftDistance = step.getDouble(MimicPropertyValue.LEFT_DISTANCE);
                    rightDistance = step.getDouble(MimicPropertyValue.RIGHT_DISTANCE);

                    angle = step.getDouble(MimicPropertyValue.ANGLE);
                }

                out.add(new MimicDriveCommand(leftPower, rightPower, leftDistance, rightDistance, angle, step.getDouble(MimicPropertyValue.BATTERY)));

                if(out.size() == 1){
                    outCommands.add(out.get(0));
                }else{
                    final ParallelCommandGroup group = new ParallelCommandGroup();
                    for(final Command com : out){
                        group.addCommands(com);
                    }
                    outCommands.add(group);
                }
            }
            return outCommands;
        };
    };

    private final BiFunction<Sequence, Double, List<Command>> generator;

    private Path(final BiFunction<Sequence, Double, List<Command>> gen){
        this.generator = gen;
    }

    public List<Command> generate(final Sequence sequence, final boolean swapped){
        final List<Command> out = new ArrayList<>();
        out.add(new RunCommand(()->{
            Robot.getRobot().getDrivebase().resetEncoders();
            Robot.getRobot().getDrivebase().disableAllPID();

            Robot.getRobot().getDrivebase().setControlMode(ControlMode.AUTO);
        }, Robot.getRobot().getDrivebase()));
        out.addAll(generator.apply(sequence, swapped ? -1.0 : 1.0));
        out.add(new RunCommand(()->{
            System.out.println("Finished path");
            Robot.getRobot().getDrivebase().resetEncoders();
            Robot.getRobot().getDrivebase().disableAutoControl();
        }, Robot.getRobot().getDrivebase()));
        return out;
    }
}