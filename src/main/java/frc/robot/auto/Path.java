package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiFunction;

import frc.robot.Robot;
import frc.robot.auto.commands.MimicDriveCommand;
import frc.robot.auto.commands.DriveToCommand;
import frc.robot.auto.commands.DriveToArcCommand;
import frc.robot.auto.commands.TurnCommand;
import frc.robot.auto.commands.WaitForTargetCommand;
import frc.robot.components.Vision.LEDState;
import frc.robot.util.ControlMode;
import frc.robot.util.Titan;

public enum Path{
    HAB1_TO_FROCKET_GEN((sequence, swapped)->{
        final Titan.ParallelCommandGroup<Robot> group = new Titan.ParallelCommandGroup<>();
        final DriveToArcCommand arc = new DriveToArcCommand(-241, -0.8, 35 * swapped);
        group.addQueue(List.of(arc, new TurnCommand(-(90-61.25) * swapped)));
        group.addQueue(List.of(new Titan.ConditionalCommand<>((rob)->arc.getProgress(rob.getDrivebase()) >= 0.3),new Titan.ConsumerCommand<>((rob)->{
            rob.getAuton().runSequence(rob, SequenceType.HATCH, sequence);
        })));
        return List.of(group);
    }),
    HAB2_TO_FROCKET_GEN((sequence, swapped)->{
        final List<Titan.Command<Robot>> outCommands = new ArrayList<>();
        outCommands.add(new DriveToArcCommand(-120, -0.5, 0));
        outCommands.add(new TurnCommand(25 * swapped));
        outCommands.add(new Titan.ConsumerCommand<>((rob)->{
            rob.getAuton().runSequence(rob, SequenceType.HATCH, sequence);
        }));
        outCommands.add(new DriveToArcCommand(-150, -0.5, 25 * swapped));
        outCommands.add(new Titan.ConsumerCommand<>((rob)->{
            rob.getVision().setLEDState(LEDState.ON);
        }));
        outCommands.add(new TurnCommand(-(90-61.25) * swapped));
        outCommands.add(new WaitForTargetCommand());
        // outCommands.add(new DriveToCommand(-40, -0.5, AutoType.DRIVE_TO));
        // outCommands.add(new DriveToArcCommand(-70, -0.5, 20 * swapped, 1.0));
        // final Titan.ParallelCommandGroup<Robot> group = new Titan.ParallelCommandGroup<>();
        // final DriveToArcCommand arc = new DriveToArcCommand(-120, -0.8, 50 * swapped);
        // // group.addQueue(List.of(arc, new TurnCommand(-(90-61.25) * swapped)));
        // // group.addQueue(List.of(new Titan.ConditionalCommand<>((rob)->arc.getProgress(rob.getDrivebase()) >= 0.3),new Titan.ConsumerCommand<>((rob)->{
        // //     rob.getAuton().runSequence(rob, SequenceType.HATCH, sequence);
        // // })));
        // //outCommands.add(arc);
        // outCommands.add(group);
        return outCommands;
    }),
    FROCKET_TO_LS_GEN((sequence, swapped)->List.of(
        new DriveToCommand(-30, -0.5),
        new TurnCommand(0),
        new Titan.ConsumerCommand<>((rob)->{
            rob.getAuton().runSequence(rob, SequenceType.HATCH, sequence);
        }),
        new DriveToArcCommand(90, 0.9, 0, 0.5),
        new DriveToArcCommand(90, 0.7, -37 * swapped),
        new DriveToArcCommand(90, 0.3, 0),
        new TurnCommand(0)
    )),
    LS_TO_CROCKET_GEN((sequence, swapped)->List.of(
        new Titan.ConsumerCommand<>((rob)->{
            rob.getAuton().runSequence(rob, SequenceType.HATCH, sequence);
        }),
        new DriveToArcCommand(-110, -0.7, 0),
        new TurnCommand((180 - 61.25) * swapped)
    )),
    HAB2_TO_SCARGO_GEN((sequence, swapped)->List.of(
        new DriveToArcCommand(-150, -0.7, 0),
        new TurnCommand(20 * swapped),
        new DriveToArcCommand(-57, -0.5, 20 * swapped),
        new Titan.ConsumerCommand<>((rob)->{
            rob.getAuton().runSequence(rob, SequenceType.HATCH, sequence);
        }),
        new TurnCommand(90 * swapped)
    )),
    SCARGO_TO_LS_GEN((sequence, swapped)->List.of(
        new DriveToArcCommand(-10, -0.7, 90 * swapped),
        new TurnCommand(0 * swapped),
        new Titan.ConsumerCommand<>((rob)->{
            rob.getAuton().runSequence(rob, SequenceType.HATCH, sequence);
        }),
        new DriveToArcCommand(20, 0.5, 0),
        new TurnCommand(-45 * swapped),
        new DriveToArcCommand(30, 0.5, -45 * swapped),
        new TurnCommand(0)
    )),
    LS_TO_SCARGO_GEN((sequence, swapped)->List.of(
        new DriveToArcCommand(-100, -0.7, 0),
        new TurnCommand(45 * swapped),
        new DriveToArcCommand(50, 0.7, 45 * swapped),
        new TurnCommand(180 * swapped),
        new DriveToArcCommand(70, 0.5, 180 * swapped),
        new TurnCommand(90 * swapped)
    )),
    HAB_TO_CCARGO_GEN((sequence, swapped)->List.of(
        new Titan.ConsumerCommand<>((rob)->{
            //Sequence.values()[stepSequence];
            rob.getAuton().runSequence(rob, SequenceType.HATCH, Sequence.ROCKET_FORWARD_1);
        }),
        new DriveToCommand(70, 0.5),
        new Titan.ConsumerCommand<>((rob)->{
            //Sequence.values()[stepSequence];
            rob.getAuton().runSequence(rob, SequenceType.HATCH, Sequence.ROCKET_FORWARD_1);
        })
    )),
    TEST(mimicGenerator("TEST"));

    private static BiFunction<Sequence, Double, List<Titan.Command<Robot>>> mimicGenerator(final String name){
        return (sequence, swapped)->{
            final List<Titan.Command<Robot>> outCommands = new ArrayList<>();

            int lastRunningSequence = -1;

            //Collect the mimic file
            //mimicChooser.getSelected()
            final List<Titan.Mimic.Step<MimicPropertyValue>> steps = Titan.Mimic.load(name, MimicPropertyValue.class);
            for(final Titan.Mimic.Step<MimicPropertyValue> step : steps){
                final List<Titan.Command<Robot>> out = new ArrayList<>();
                if(step.getBoolean(MimicPropertyValue.HOME)){
                    out.add(new Titan.ConsumerCommand<>((rob)->{
                        rob.getDrivebase().reset();
                    }));
                }

                final int stepSequence = step.getInteger(MimicPropertyValue.RUNNING_SEQUENCE);
                if(sequence != null && stepSequence >= 0 && stepSequence != lastRunningSequence){
                    out.add(new Titan.ConsumerCommand<>((rob)->{
                        //Sequence.values()[stepSequence];
                        rob.getAuton().runSequence(rob, SequenceType.values()[step.getInteger(MimicPropertyValue.SEQUENCE_TYPE)], sequence);
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
                    final Titan.ParallelCommandGroup<Robot> group = new Titan.ParallelCommandGroup<>();
                    for(final Titan.Command<Robot> com : out){
                        group.addCommand(com);
                    }
                    outCommands.add(group);
                }
            }
            return outCommands;
        };
    };

    private final BiFunction<Sequence, Double, List<Titan.Command<Robot>>> generator;

    private Path(final BiFunction<Sequence, Double, List<Titan.Command<Robot>>> gen){
        this.generator = gen;
    }

    public List<Titan.Command<Robot>> generate(final Sequence sequence, final boolean swapped){
        final List<Titan.Command<Robot>> out = new ArrayList<>();
        out.add(new Titan.ConsumerCommand<>((rob)->{
            rob.getDrivebase().resetEncoders();
            rob.getDrivebase().disableAllPID();

            rob.getDrivebase().setControlMode(ControlMode.AUTO);
        }));
        out.addAll(generator.apply(sequence, swapped ? -1.0 : 1.0));
        out.add(new Titan.ConsumerCommand<>((rob)->{
            System.out.println("Finished path");
            rob.getDrivebase().resetEncoders();
            rob.getDrivebase().disableAutoControl();
        }));
        return out;
    }
}