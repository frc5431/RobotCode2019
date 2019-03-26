package frc.robot.util;

import frc.robot.Robot;
import frc.robot.util.Testable;

public abstract class Component implements Testable{
    public abstract void init(final Robot robot);

    public abstract void periodic(final Robot robot);

    public abstract void disabled(final Robot robot);

    public void tick(final Robot robot){
        //do nothing
    }
}