package frc.robot.auto.vision;

public class TargetInfo{
    private boolean exists;
    private double xAngle, yAngle, area;

    public TargetInfo(final boolean exists, final double xAngle, final double yAngle, final double area){
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