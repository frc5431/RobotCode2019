package frc.robot.auto.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public enum Limelight{
    FRONT(false, "limelight-front", false);
    
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