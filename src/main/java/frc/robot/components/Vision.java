package frc.robot.components;

import frc.robot.util.Component;
import frc.robot.util.Testable;
import frc.robot.util.ListenerThread;
import frc.robot.Robot;
import frc.robot.util.Titan;

import edu.wpi.first.wpilibj.DigitalOutput;

public class Vision extends Component{
    public static enum LEDState{
        ON, OFF
    };

    private final double[] distances = new double[]{0, 0, 0};

    private final DigitalOutput led;
    private LEDState ledState = LEDState.OFF;

    public Vision(){
        new ListenerThread(5431, (message)->{
            final String[] comps = message.split(",", 3);
            if(!comps[0].equals("null") && !comps[0].equals("nan")){
                distances[0] = Double.parseDouble(comps[0]);
            }
            if(!comps[1].equals("null") && !comps[1].equals("nan")){
                distances[1] = Double.parseDouble(comps[1]);
            }
            if(!comps[2].equals("null") && !comps[2].equals("nan")){
                distances[2] = Double.parseDouble(comps[2]);
            }

            //System.out.println(message);
        }).start();
        led = new DigitalOutput(2);
        //led = new Relay(1, Relay.Direction.kBoth);
    }

    @Override
    public void init(final Robot robot){
        ledState = LEDState.OFF;
    }

    @Override
    public void periodic(final Robot robot){
        led.set(ledState == LEDState.ON || robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.A));
       //led.set(ledState == LEDState.ON || robot.getTeleop().getDriver().getRawButton(Titan.Xbox.Button.A) ? Value.kOn : Value.kOff);
    }

    @Override
    public void disabled(final Robot robot){
        ledState = LEDState.OFF;
    }

    public double[] getDistancesToTarget(){
        return distances;
    }

    public void setLEDState(final LEDState state){
        ledState = state;
    }

    public LEDState getLEDState(){
        return ledState;
    }

    @Override
    public String getTestResult(){
        return Testable.SUCCESS;
    }
}