package frc.robot.components;

import frc.robot.util.Component;
import frc.robot.util.Testable;
import frc.robot.Robot;
import frc.robot.util.Titan;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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

    public static enum TargetingDirection{
        FRONT, BACK
    };

    public static enum LEDState{
        ON, OFF
    };

    private final NetworkTable frontLimelightTable, backLimelightTable;
    private LEDState ledState = LEDState.OFF;
    private TargetingDirection direction = TargetingDirection.FRONT;

    public Vision(){
        frontLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        backLimelightTable = NetworkTableInstance.getDefault().getTable("backLimelight");
    }

    private NetworkTable getSelectedTable(){
        switch(direction){
            case BACK:
                return backLimelightTable;
            case FRONT:
            default:
                return frontLimelightTable;
        }
    }

    @Override
    public void init(final Robot robot){
        ledState = LEDState.OFF;
        direction = TargetingDirection.FRONT;
    }

    @Override
    public void periodic(final Robot robot){
    }

    @Override
    public void tick(final Robot robot){
        if(robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.X)){
            frontLimelightTable.getEntry("ledMode").setNumber(2);
            backLimelightTable.getEntry("ledMode").setNumber(2);
        }else{
            frontLimelightTable.getEntry("ledMode").setNumber((direction == TargetingDirection.FRONT && ledState == LEDState.ON) || robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.A) ? 3 : 1);
            backLimelightTable.getEntry("ledMode").setNumber((direction == TargetingDirection.BACK && ledState == LEDState.ON) || robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.B) ? 3 : 1);
        }
    }

    @Override
    public void disabled(final Robot robot){
        ledState = LEDState.OFF;
    }

    public TargetInfo getTargetInfo(){
        final NetworkTable table = getSelectedTable();
        return new TargetInfo(table.getEntry("tv").getBoolean(false), table.getEntry("tx").getDouble(0), table.getEntry("ty").getDouble(0), table.getEntry("ta").getDouble(0));
    }

    public void setLEDState(final LEDState state){
        ledState = state;
    }

    public LEDState getLEDState(){
        return ledState;
    }

    public void setDirection(final TargetingDirection dir){
        this.direction = dir;
    }

    public TargetingDirection getDirection(){
        return direction;
    }

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}