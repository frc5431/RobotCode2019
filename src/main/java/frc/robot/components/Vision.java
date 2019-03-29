package frc.robot.components;

import frc.robot.util.Component;
import frc.robot.util.Testable;
import frc.robot.Robot;
import frc.robot.util.Titan;

import frc.robot.auto.vision.TargetInfo;
import frc.robot.auto.vision.TargetType;
import frc.robot.auto.vision.Limelight;

import edu.wpi.first.networktables.NetworkTable;

public class Vision extends Component{
    public static enum LEDState{
        ON, OFF
    };

    private LEDState ledState = LEDState.OFF;
    private TargetType ttype = TargetType.FRONT_RIGHT;

    public Vision(){
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
        if(robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.X)){
            for(final Limelight l : Limelight.values()){
                l.getTable().getEntry("ledMode").setNumber(2);
            }
        }else{
            Limelight.FRONT.getTable().getEntry("ledMode").setNumber((ttype.getLimelight() == Limelight.FRONT && ledState == LEDState.ON) || robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.A) ? 3 : 1);
            Limelight.BACK.getTable().getEntry("ledMode").setNumber((ttype.getLimelight() == Limelight.BACK && ledState == LEDState.ON) || robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.B) ? 3 : 1);
        }

        getSelectedTable().getEntry("pipeline").setNumber(ttype.getPipeline());
    }

    @Override
    public void disabled(final Robot robot){
        ledState = LEDState.OFF;
    }

    
    private NetworkTable getSelectedTable(){
        return ttype.getLimelight().getTable();
    }

    public TargetInfo getTargetInfo(){
        final NetworkTable table = getSelectedTable();
        final double ts = table.getEntry("ts").getDouble(0);
        if(ts > -60 && ts < -10){
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

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}