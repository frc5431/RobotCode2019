package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Titan;

public class Intake{
    private final WPI_TalonSRX rollers;

    private final Solenoid hatchLeft, hatchRight, finger;

    private final Titan.Lidar hatchLidar, ballLidar;

    private boolean isHatching = true, isFingering = true;

    public Intake(){
        rollers = new WPI_TalonSRX(Constants.INTAKE_ROLLER_ID);
        rollers.setInverted(Constants.INTAKE_ROLLER_INVERTED);
        rollers.setNeutralMode(NeutralMode.Brake);

        hatchLeft = new Solenoid(Constants.INTAKE_HATCH_LEFT_PCM_ID, Constants.INTAKE_HATCH_LEFT_ID);
        hatchRight = new Solenoid(Constants.INTAKE_HATCH_RIGHT_PCM_ID, Constants.INTAKE_HATCH_RIGHT_ID);
        finger = new Solenoid(Constants.INTAKE_FINGER_PCM_ID, Constants.INTAKE_FINGER_ID);

        hatchLidar = new Titan.Lidar(Constants.INTAKE_HATCH_LIDAR_PORT);
        hatchLidar.setCalibrationOffset(7);

        ballLidar = new Titan.Lidar(Constants.INTAKE_BALL_LIDAR_PORT);
    }

    public void periodic(final Robot robot){
        hatchLeft.set(!isHatching);
        hatchRight.set(!isHatching);

        finger.set(!isFingering);
    }

    public void roll(final double val){
        rollers.set(val);
    }

    public void actuateHatch(final boolean val){
        isHatching = val;
    }

    public void finger(final boolean val){
        isFingering = val;
    }

    public boolean isFingering(){
        return isFingering;
    }

    public boolean isHatchOuttaking(){
        return isHatching;
    }

    public double getBallDistance(){
        return ballLidar.getDistance();
    }

    public double getHatchDistance(){
        return hatchLidar.getDistance();
    }

    public boolean isBallIn(){
        return false;
    }

    public boolean canShootHatch(){
        return true;
    }
}