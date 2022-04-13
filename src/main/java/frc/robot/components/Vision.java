package frc.robot.components;

import frc.robot.Robot;
import frc.team5431.titan.core.joysticks.Xbox;
import frc.robot.auto.SequenceType;
import frc.robot.auto.vision.TargetInfo;
import frc.robot.auto.vision.TargetType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public static enum LEDState{
        ON, OFF
    };

    private final NetworkTable table;
    private LEDState ledState = LEDState.OFF;
    private TargetType ttype = TargetType.FRONT_RIGHT;

    public Vision(){
        table = NetworkTableInstance.getDefault().getTable("limelight-front");

        ledState = LEDState.OFF;
        ttype = TargetType.FRONT_RIGHT;
    }

    @Override
    public void periodic() {
        final NetworkTable selectedTable = getSelectedTable();

        final Xbox controller = Robot.getRobot().getTeleop().getDriver();
        if(controller.getRawButton(Xbox.Button.X)){
            selectedTable.getEntry("ledMode").setNumber(2);
        }else{
            selectedTable.getEntry("ledMode").setNumber(ledState == LEDState.ON || controller.getRawButton(Xbox.Button.A) || Robot.getRobot().getAuton().getCurrentSequenceType() == SequenceType.HATCH ? 3 : 1);
        }

        selectedTable.getEntry("pipeline").setNumber(ttype.getPipeline());
    }
    
    private NetworkTable getSelectedTable(){
        return table;
    }

    public TargetInfo getTargetInfo(){
        final NetworkTable table = getSelectedTable();
        final double ts = table.getEntry("ts").getDouble(0);
        if(ts > -87 && ts < -10){
            return new TargetInfo(false, 0, 0, 0);
        }
        return new TargetInfo(table.getEntry("tv").getDouble(0) == 1.0, table.getEntry("tx").getDouble(0), table.getEntry("ty").getDouble(0), table.getEntry("ta").getDouble(0));
    }

    public void setLEDState(final LEDState state){
        ledState = state;
    }

    public LEDState getLEDState(){
        return ledState;
    }

    public void setTargetType(final TargetType ttype){
        this.ttype = ttype;
    }

    public TargetType getTargetType(){
        return ttype;
    }
}