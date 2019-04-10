package frc.robot.util;

public class PIDConstants{
    private final double p, i, d;

    public PIDConstants(final double p, final double i, final double d){
        //big
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double getP(){
        return p;
    }

    public double getI(){
        return i;
    }

    public double getD(){
        return d;
    }
}