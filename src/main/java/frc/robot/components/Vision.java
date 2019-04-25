package frc.robot.components;

import frc.robot.Robot;
import frc.robot.util.Titan;
import frc.robot.auto.SequenceType;
import frc.robot.auto.vision.TargetInfo;
import frc.robot.auto.vision.TargetType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision extends Titan.Component<Robot>{
    public static enum LEDState{
        ON, OFF
    };

    private final NetworkTable table;
    private LEDState ledState = LEDState.OFF;
    private TargetType ttype = TargetType.FRONT_RIGHT;

    public Vision(){
        table = NetworkTableInstance.getDefault().getTable("limelight-front");
    }

    @Override
    public void init(final Robot robot){
        ledState = LEDState.OFF;
        ttype = TargetType.FRONT_RIGHT;
    }

    @Override
    public void periodic(final Robot robot){
    }

    @Override
    public void tick(final Robot robot){
        final NetworkTable selectedTable = getSelectedTable();

        final Titan.Xbox controller = robot.getTeleop().getDriver();
        if(controller.getRawButton(Titan.Xbox.Button.X)){
            selectedTable.getEntry("ledMode").setNumber(2);
        }else{
            selectedTable.getEntry("ledMode").setNumber(ledState == LEDState.ON || controller.getRawButton(Titan.Xbox.Button.A) || robot.getAuton().getCurrentSequenceType() == SequenceType.HATCH ? 3 : 1);
        }

        selectedTable.getEntry("pipeline").setNumber(ttype.getPipeline());
    }

    @Override
    public void disabled(final Robot robot){
        ledState = LEDState.OFF;
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