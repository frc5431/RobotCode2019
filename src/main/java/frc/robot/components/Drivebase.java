package frc.robot.components;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Drivebase {
    private final CANSparkMax frontLeft, frontRight, backLeft, backRight;

    public Drivebase(){
        frontLeft = new CANSparkMax(Constants.DRIVEBASE_FRONT_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontLeft.setInverted(Constants.DRIVEBASE_FRONT_LEFT_INVERTED);
        
        frontRight = new CANSparkMax(Constants.DRIVEBASE_FRONT_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight.setInverted(Constants.DRIVEBASE_FRONT_RIGHT_INVERTED);
        
        backLeft = new CANSparkMax(Constants.DRIVEBASE_BACK_LEFT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backLeft.setInverted(Constants.DRIVEBASE_BACK_LEFT_INVERTED);

        //he attac
        //he protec
        //but most importantly
        //he bac
        backRight = new CANSparkMax(Constants.DRIVEBASE_BACK_RIGHT_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        backRight.setInverted(Constants.DRIVEBASE_BACK_RIGHT_INVERTED);
    }

    public void driveLeft(final double val){
        frontLeft.set(val);
        backLeft.set(val);
    }

    public void driveRight(final double val){
        frontRight.set(val);
        backRight.set(val);
    }

    public void drive(final double left, final double right){
        driveLeft(left);
        driveRight(right);
    }
}