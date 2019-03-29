package frc.robot.auto.motionprofile;

/**
 *
 * @author kyledematias
 */
import java.util.ArrayList;
        
public abstract class Profile {
    
    public static Profile getVelProfile(final double max_velocity, final double acceleration, final double distance, final double start_vel, final double end_vel){
        assert acceleration == 0.0 && max_velocity == start_vel && start_vel == end_vel: "Velocity must stay constant (ie. accel = 0)";
        if((Math.pow(max_velocity, 2.0) / acceleration) < Math.abs(distance) || acceleration == 0.0){
            if(distance < 0.0){
                return new TrapProfile(-max_velocity, acceleration, distance, -start_vel, -end_vel);
            }
			return new TrapProfile(max_velocity, acceleration, distance, start_vel, end_vel);
        }
		if(distance < 0.0){
		    return new TriProfile(-max_velocity, acceleration, distance, -start_vel, -end_vel);
		}
		return new TriProfile(max_velocity, acceleration, distance, start_vel, end_vel);
    }
   
    public abstract double getFinalTime();
    public abstract double getVelocityAtTime(final double time);
    public abstract double getDistAtTime(final double time);
    public abstract ArrayList<Double> getDistances(); //each 100 ms increment
    public abstract double getDist();
    public abstract double getMaxVel();
    public abstract double getAccel();
    public abstract double getStartVel();
    public abstract double getEndVel();
}