package frc.robot.components;

import frc.robot.util.Component;
import frc.robot.util.Testable;
import frc.robot.Robot;
import frc.robot.util.Titan;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends Component{
    public static class TargetInfo{
        private boolean exists;
        private double xAngle, yAngle, area;

        TargetInfo(final boolean exists, final double xAngle, final double yAngle, final double area){
            this.exists = exists;
            this.xAngle = xAngle;
            this.yAngle = yAngle;
            this.area = area;
        }

        public boolean exists(){
            return exists;
        }

        public double getXAngle(){
            return xAngle;
        }

        public double getYAngle(){
            return yAngle;
        }

        public double getArea(){
            return area;
        }
    };

    public static enum LEDState{
        ON, OFF
    };

    public static enum Limelight{
        FRONT(false, "limelight-front", false), BACK(true, "limelight-back", true);
        
        private final NetworkTable table;
        private final boolean centered, inverted;

        private Limelight(final boolean centered, final String name, final boolean inverted){
            this.centered = centered;
            this.table = NetworkTableInstance.getDefault().getTable(name);
            this.inverted = inverted;
        }

        public boolean isCentered(){
            return centered;
        }

        public boolean isInverted(){
            return inverted;
        }

        public NetworkTable getTable(){
            return table;
        }
    };

    public static enum TargetType{
        FRONT_RIGHT(Limelight.FRONT, 0), FRONT_LEFT(Limelight.FRONT, 1), BACK(Limelight.BACK, 0);

        private final Limelight llight;
        private final int pipeline;

        private TargetType(final Limelight light, final int pipe){
            this.llight = light;
            this.pipeline = pipe;
        }

        public Limelight getLimelight(){
            return llight;
        }

        public int getPipeline(){
            return pipeline;
        }
    };

    private LEDState ledState = LEDState.OFF;
    private TargetType ttype = TargetType.FRONT_RIGHT;

    public Vision(){

    }

    private NetworkTable getSelectedTable(){
        return ttype.getLimelight().getTable();
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

    public TargetInfo getTargetInfo(){
        final NetworkTable table = getSelectedTable();
        final double ts = table.getEntry("ts").getDouble(0);
        if(ts > -60 && ts < -10){
            System.out.println("SKEW: " + table.getEntry("ts").getDouble(0));
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